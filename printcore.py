# Klipper extras: print core amovível, com MAX31865 (PT1000) e ID por resistência
#
# Princípio: a proteção normal do Klipper fica SEMPRE ativa. Só é aliviada
# dentro de um "modo de troca" explícito, e só se repõe quando a troca é
# validada.
#
#   Fora do modo de troca : comportamento 100% stock. Se o print core for
#                           retirado, o MCU faz shutdown ("Thermocouple reader
#                           fault"), como em qualquer Klipper.
#   PRINTCORE_GOTO_SWAP   : X= Y= Z= [SPEED= Z_SPEED=] leva a cabeça à posição
#                           de troca (a tabela está na macro PRINT_CORE_CHANGE).
#   PRINTCORE_SWAP_START  : (mesmos X= Y= Z=) só aceita sem impressão, com XYZ
#                           em home, aquecedor desligado, PWM a zero e hotend
#                           frio. Vai à posição
#                           de troca, pára as leituras do MAX31865 no MCU (é
#                           isto que evita o shutdown), bloqueia o aquecimento
#                           E O MOVIMENTO (G28, G0/G1, probes, BEACON_*, ... e
#                           qualquer move do toolhead). Confirma que as
#                           leituras pararam.
#   PRINTCORE_SWAP_END    : exige ID de nozzle válido e estável, limpa as
#                           falhas do MAX31865, faz leituras de teste diretas
#                           por SPI (sem falha, temperatura plausível, estável)
#                           e só então rearma as leituras do MCU, com os mesmos
#                           limites que o Klipper usa. Confirma que chegam
#                           leituras. Qualquer falha em confirmar = shutdown.
#                           Só então desbloqueia o movimento e, por omissão
#                           (rehome_after_swap), invalida o home de XYZ.
#   Timeout               : se a troca não terminar a tempo, shutdown.
#
# Porque é seguro estar sem leituras durante a troca: o PWM do aquecedor só é
# atualizado quando chega uma leitura de temperatura (Heater.set_pwm), e o
# aquecedor tem de estar a zero para entrar no modo. Sem leituras o aquecedor
# não pode ligar. O bloqueio de set_temp acrescenta uma segunda barreira.
#
# Instalação: copiar para klipper/klippy/extras/printcore.py e reiniciar.
# Só funciona com sensor_type: MAX31865 no aquecedor. NÃO TESTADO EM HARDWARE.

import inspect
import logging
from . import spi_temperature

REPORT_TIME = 0.300
ADC_SAMPLE_TIME = 0.001
ADC_SAMPLE_COUNT = 8

ABSENT = "absent"      # ID em aberto
UNKNOWN = "unknown"    # algo ligado, mas o ID não corresponde a nenhum nozzle
PRESENT = "present"    # print core reconhecido

# Comandos que movem a máquina (ou alteram o estado de "homed") e que ficam
# recusados durante a troca. É uma 1.ª barreira, com mensagem clara e SEM
# efeitos secundários (ex.: o G28 do homing_override mexe em SET_KINEMATIC_
# POSITION e correntes dos motores antes do primeiro movimento). Só se
# envolvem os que existirem. A 2.ª barreira está no próprio toolhead.
BLOCKED_COMMANDS = (
    "G28", "G0", "G1", "G2", "G3", "G29",
    "SET_KINEMATIC_POSITION", "FORCE_MOVE", "STEPPER_BUZZ", "MANUAL_STEPPER",
    "PROBE", "PROBE_ACCURACY", "PROBE_CALIBRATE",
    "BEACON_AUTO_CALIBRATE", "BEACON_AUTO_CALIBRATE_OLD",
    "BEACON_CALIBRATE", "BEACON_OFFSET_COMPARE", "BEACON_POKE",
    "BED_MESH_CALIBRATE", "Z_TILT_ADJUST", "QUAD_GANTRY_LEVEL",
    "SCREWS_TILT_CALCULATE", "Z_ENDSTOP_CALIBRATE",
    "AXIS_TWIST_COMPENSATION_CALIBRATE", "TEST_RESONANCES",
    "SHAPER_CALIBRATE", "MEASURE_AXES_NOISE", "HOME_IF_NEEDED",
)

QUERY_FMT = ("query_thermocouple oid=%c clock=%u rest_ticks=%u"
             " min_value=%u max_value=%u max_invalid_count=%c")

RTD_MSB_REG = 0x01
FAULT_STATUS_REG = 0x07


class PrintCore:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.heater_name = config.get('heater', 'extruder')
        self.heater = None
        self.sensor = None
        self.mcu = None
        self.query_cmd = None
        self._faulted = False

        # --- troca ---
        self.safe_temp = config.getfloat('safe_temp', 50., above=0.)
        # Limite inferior próprio: o min_temp do extruder pode ser muito baixo
        # (ex.: -273) e não serve para validar a leitura no fim da troca.
        self.swap_min_temp = config.getfloat('swap_min_temp', 0.)
        self.swap_timeout = config.getfloat('swap_timeout', 600., minval=0.)
        self.swap_samples = config.getint('swap_samples', 5, minval=2)
        self.swap_max_spread = config.getfloat('swap_max_spread', 2., above=0.)
        # Depois de trocar, o nozzle pode ter outra altura: obriga a novo home
        self.rehome_after_swap = config.getboolean('rehome_after_swap', True)
        extra = config.get('swap_block_extra', '')
        self.block_cmds = list(BLOCKED_COMMANDS) + [
            c.strip().upper() for c in extra.replace('\n', ',').split(',')
            if c.strip()]
        self.swap_active = False
        self.swap_timer = self.reactor.register_timer(
            self._swap_timeout_cb, self.reactor.NEVER)

        # --- ID do nozzle: leitura do ADC em "counts" de 12 bits (0-4096),
        # a mesma escala da tabela de códigos (V / (3.3 / 4096)) ---
        self.tol = config.getfloat('id_tolerance', 15., above=0.)
        self.absent_counts = config.getfloat('absent_counts', 3800.,
                                             above=0., maxval=4096.)
        self.debounce = config.getint('debounce_samples', 3, minval=1)
        self.nozzles = self._parse_nozzles(config)
        self.state = ABSENT
        self.nozzle = ""
        self.counts = None
        self._cand = None
        self._cand_count = 0

        ppins = self.printer.lookup_object('pins')
        self.id_adc = ppins.setup_pin('adc', config.get('id_pin'))
        # ADC do ID sem verificação de limites no MCU: ID em aberto é um
        # estado normal (print core retirado), não uma falha.
        self._setup_id_adc(self.id_adc)

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler("klippy:ready",
                                            self._handle_ready)
        for name, func, desc in (
                ("PRINTCORE_STATUS", self.cmd_STATUS,
                 "Estado do print core"),
                ("PRINTCORE_REQUIRE", self.cmd_REQUIRE,
                 "Falha se o print core/nozzle não for o esperado"),
                ("PRINTCORE_GOTO_SWAP", self.cmd_GOTO_SWAP,
                 "Leva a cabeça à posição de troca (sem bloquear nada)"),
                ("PRINTCORE_SWAP_START", self.cmd_SWAP_START,
                 "Inicia a troca do print core (alivia a proteção)"),
                ("PRINTCORE_SWAP_END", self.cmd_SWAP_END,
                 "Termina a troca e repõe a proteção normal")):
            self.gcode.register_command(name, func, desc=desc)

    # ------------------------------------------------------------------
    def _parse_nozzles(self, config):
        # Uma por linha:   nome: counts   (valor médio medido no ADC)
        nozzles = []
        for line in config.get('nozzles').split('\n'):
            line = line.strip()
            if not line:
                continue
            try:
                name, counts = line.rsplit(':', 1)
                nozzles.append((name.strip(), float(counts)))
            except ValueError:
                raise config.error("printcore: linha inválida em nozzles: "
                                   "'%s' (esperado 'nome: counts')" % (line,))
        if not nozzles:
            raise config.error("printcore: nozzles vazio")
        nozzles.sort(key=lambda n: n[1])
        # As janelas [counts - tol, counts + tol] não podem sobrepor-se
        for (na, ca), (nb, cb) in zip(nozzles, nozzles[1:]):
            if ca + self.tol >= cb - self.tol:
                raise config.error(
                    "printcore: IDs '%s' (%.0f) e '%s' (%.0f) sobrepõem-se "
                    "com id_tolerance=%.0f" % (na, ca, nb, cb, self.tol))
        top_name, top = nozzles[-1]
        if top + self.tol >= self.absent_counts:
            raise config.error(
                "printcore: '%s' (%.0f) demasiado perto de absent_counts "
                "(%.0f)" % (top_name, top, self.absent_counts))
        return nozzles

    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        sensor = self.heater.sensor
        if not isinstance(sensor, spi_temperature.MAX31865):
            raise self.printer.config_error(
                "printcore: o aquecedor '%s' tem de usar sensor_type: "
                "MAX31865" % (self.heater_name,))
        self.sensor = sensor
        self.mcu = sensor.mcu
        self.query_cmd = self.mcu.lookup_command(QUERY_FMT)

        # Bloqueia pedidos de temperatura > 0 em modo de troca ou sem ID
        # válido (cobre M104, M109, SET_HEATER_TEMPERATURE, macros...)
        orig_set_temp = self.heater.set_temp

        def guarded_set_temp(degrees):
            if degrees > 0. and not self.heating_allowed():
                raise self.printer.command_error(
                    "printcore: aquecimento bloqueado (troca_ativa=%s, "
                    "estado=%s)" % (self.swap_active, self.state))
            return orig_set_temp(degrees)

        self.heater.set_temp = guarded_set_temp

        # alter_target (usado pelo PID_CALIBRATE) também muda o alvo
        orig_alter_target = self.heater.alter_target

        def guarded_alter_target(target_temp):
            if target_temp > 0. and not self.heating_allowed():
                raise self.printer.command_error(
                    "printcore: aquecimento bloqueado (troca_ativa=%s, "
                    "estado=%s)" % (self.swap_active, self.state))
            return orig_alter_target(target_temp)

        self.heater.alter_target = guarded_alter_target

    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # Bloqueio de movimento durante a troca
    def _handle_ready(self):
        gcode = self.gcode
        wrapped = []
        for name in self.block_cmds:
            old = gcode.register_command(name, None)
            if old is None:
                continue
            gcode.register_command(name, self._make_block(name, old))
            wrapped.append(name)
        logging.info("printcore: comandos com bloqueio na troca: %s",
                     ", ".join(wrapped))
        # 2.ª barreira: qualquer movimento que chegue ao toolhead (macros,
        # transforms, extrusor, homing). Atributos de instância por cima dos
        # métodos; quem chama self.move()/self.drip_move() passa aqui.
        toolhead = self.printer.lookup_object('toolhead')
        for attr in ('move', 'drip_move', 'manual_move'):
            setattr(toolhead, attr, self._make_guard(getattr(toolhead, attr)))

    def _motion_error(self):
        return ("printcore: movimento bloqueado - troca do print core em "
                "curso. Monta o print core e corre PRINT_CORE_CHANGE_DONE")

    def _make_block(self, name, old):
        def blocked(gcmd):
            if self.swap_active:
                raise gcmd.error("%s (%s)" % (self._motion_error(), name))
            return old(gcmd)
        return blocked

    def _make_guard(self, orig):
        def guard(*args, **kwargs):
            if self.swap_active:
                raise self.printer.command_error(self._motion_error())
            return orig(*args, **kwargs)
        return guard

    def _check_idle(self, gcmd):
        ps = self.printer.lookup_object('print_stats', None)
        if ps is not None:
            state = ps.get_status(self.reactor.monotonic()).get('state')
            if state in ('printing', 'paused'):
                raise gcmd.error("printcore: há uma impressão em curso "
                                 "(%s); cancela-a antes de trocar" % state)

    def _swap_params(self, gcmd):
        # Fonte única: tabela [gcode_macro PRINT_CORE_POSITIONS]. Parâmetros
        # no comando (X Y Z SPEED Z_SPEED) têm prioridade sobre a tabela.
        # Coordenadas da máquina.
        tbl = {}
        macro = self.printer.lookup_object(
            'gcode_macro PRINT_CORE_POSITIONS', None)
        if macro is not None:
            tbl = getattr(macro, 'variables', {}) or {}

        def pick(param, var, default=None):
            if gcmd.get(param, None) is not None:
                return None  # vem do comando
            if var in tbl:
                try:
                    return float(tbl[var])
                except (TypeError, ValueError):
                    raise gcmd.error(
                        "printcore: %s em PRINT_CORE_POSITIONS não é um "
                        "número: %r" % (var, tbl[var]))
            if default is not None:
                return default
            raise gcmd.error(
                "printcore: falta %s: passa-o no comando ou define %s em "
                "[gcode_macro PRINT_CORE_POSITIONS]" % (param, var))

        def val(param, var, default=None, **kw):
            t = pick(param, var, default)
            if t is None:
                return gcmd.get_float(param, **kw)
            lo, above = kw.get('minval'), kw.get('above')
            if (lo is not None and t < lo) or (above is not None
                                               and t <= above):
                raise gcmd.error("printcore: %s=%s inválido em "
                                 "PRINT_CORE_POSITIONS" % (var, t))
            return t

        x = val('X', 'swap_x')
        y = val('Y', 'swap_y')
        z = val('Z', 'swap_z', minval=0.)
        speed = val('SPEED', 'speed_xy', 150., above=0.)
        z_speed = val('Z_SPEED', 'speed_z', 10., above=0.)
        return x, y, z, speed, z_speed

    def _goto_swap_pos(self, gcmd):
        x, y, z, speed, z_speed = self._swap_params(gcmd)
        th = self.printer.lookup_object('toolhead')
        status = th.get_status(self.reactor.monotonic())
        homed = status['homed_axes']
        if not all(a in homed for a in "xyz"):
            raise gcmd.error("printcore: faz home a XYZ antes de ir para a "
                             "posição de troca (homed='%s')" % (homed,))
        cur = th.get_position()
        # Sobe antes de andar em XY; se já está acima, anda e só depois desce
        if cur[2] < z:
            th.manual_move([None, None, z], z_speed)
        th.manual_move([x, y, None], speed)
        th.manual_move([None, None, z], z_speed)
        th.wait_moves()
        # manual_move não passa pelo gcode_move: ressincroniza-o
        self.printer.lookup_object('gcode_move').reset_last_position()
        pos = th.get_position()
        if abs(pos[0] - x) > 0.05 or abs(pos[1] - y) > 0.05 \
                or abs(pos[2] - z) > 0.05:
            raise gcmd.error("printcore: a cabeça não ficou na posição de "
                             "troca (%.2f, %.2f, %.2f)" % tuple(pos[:3]))
        return x, y, z

    def cmd_GOTO_SWAP(self, gcmd):
        if self.swap_active:
            raise gcmd.error("printcore: troca já em curso")
        self._check_idle(gcmd)
        x, y, z = self._goto_swap_pos(gcmd)
        gcmd.respond_info("Cabeça na posição de troca X%.1f Y%.1f Z%.1f"
                          % (x, y, z))

    # ------------------------------------------------------------------
    def heating(self):
        return self.heater is not None and self.heater.target_temp > 0.

    def heating_allowed(self):
        return (not self.swap_active and self.state == PRESENT
                and not self._faulted)

    def fault(self, msg):
        if self._faulted:
            return
        self._faulted = True
        logging.error("printcore: %s", msg)
        self.printer.invoke_shutdown("Print core: " + msg)

    # ------------------------------------------------------------------
    # ID do nozzle
    def _classify(self, counts):
        # Só é válido se cair DENTRO da janela de um nozzle da tabela. Fora
        # das janelas (incluindo ~0, que é igual a um curto-circuito do ID)
        # fica "unknown" e o aquecimento fica bloqueado.
        if counts >= self.absent_counts:
            return ABSENT, ""
        for name, c in self.nozzles:
            if abs(counts - c) <= self.tol:
                return PRESENT, name
        return UNKNOWN, ""

    def _setup_id_adc(self, adc):
        # A API do MCU_adc mudou entre versões do Klipper. Escolhe-se pela
        # assinatura real em vez de assumir uma versão.
        def params(fn):
            return list(inspect.signature(fn).parameters)
        if hasattr(adc, 'setup_adc_sample'):
            sample_params = params(adc.setup_adc_sample)
        elif hasattr(adc, 'setup_minmax'):
            sample_params = params(adc.setup_minmax)
        else:
            raise self.printer.config_error(
                "printcore: este Klipper não tem API de ADC conhecida")
        cb_params = params(adc.setup_adc_callback)
        new_api = bool(sample_params) and sample_params[0] == 'report_time'
        sample_fn = getattr(adc, 'setup_adc_sample', None) \
            or adc.setup_minmax
        if new_api:
            sample_fn(REPORT_TIME, ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT,
                      minval=0., maxval=1.)
        else:
            sample_fn(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT,
                      minval=0., maxval=1.)
        if cb_params and cb_params[0] == 'report_time':
            adc.setup_adc_callback(REPORT_TIME, self._id_callback)
        else:
            adc.setup_adc_callback(self._id_callback)

    def _id_callback(self, read_time, read_value):
        counts = read_value * 4096.
        cand = self._classify(counts)
        if cand == self._cand:
            self._cand_count += 1
        else:
            self._cand = cand
            self._cand_count = 1
        if (self._cand_count >= self.debounce
                and cand != (self.state, self.nozzle)):
            self._commit(cand[0], cand[1], counts)

    def _commit(self, state, nozzle, counts):
        prev = (self.state, self.nozzle)
        self.state, self.nozzle, self.counts = state, nozzle, counts
        if self.heating():
            self.fault("ID do print core mudou de '%s/%s' para '%s/%s' com o "
                       "aquecedor ativo" % (prev + (state, nozzle)))
            return
        if state == PRESENT:
            self.gcode.respond_info("Print core detetado: %s (ADC %.0f)"
                                    % (nozzle, counts))
        elif state == ABSENT:
            self.gcode.respond_info("Print core removido")
        else:
            self.gcode.respond_info(
                "!! Print core com ID desconhecido (ADC %.0f): aquecimento "
                "bloqueado" % (counts,))

    # ------------------------------------------------------------------
    # Controlo das leituras do MAX31865 no MCU
    def _next_clock(self, delay=0.3):
        pt = self.mcu.estimated_print_time(self.reactor.monotonic()) + delay
        return self.mcu.print_time_to_clock(pt)

    def _query(self, rest_ticks):
        s = self.sensor
        self.query_cmd.send([s.oid, self._next_clock(), rest_ticks,
                             s.min_sample_value, s.max_sample_value,
                             spi_temperature.MAX_INVALID_COUNT])

    def _stop_polling(self):
        self._query(0)      # rest_ticks=0: o MCU pára o timer de leitura

    def _start_polling(self):
        # Mesmos min/max/max_invalid que o Klipper usa no arranque
        self._query(self.mcu.seconds_to_clock(spi_temperature.REPORT_TIME))

    def _pause(self, secs):
        self.reactor.pause(self.reactor.monotonic() + secs)

    def _read_rtd(self):
        # Leitura direta por SPI, igual à do firmware (thermocouple.c)
        spi = self.sensor.spi
        resp = bytearray(spi.spi_transfer([RTD_MSB_REG, 0, 0])['response'])
        value = (resp[1] << 8) | resp[2]
        resp = bytearray(spi.spi_transfer([FAULT_STATUS_REG, 0])['response'])
        fault = (resp[1] & ~0x03 & 0xff) | (value & 0x0001)
        return value, fault

    # ------------------------------------------------------------------
    # Modo de troca
    def cmd_SWAP_START(self, gcmd):
        if self.heater is None:
            raise gcmd.error("printcore: aquecedor ainda não disponível")
        if self.swap_active:
            raise gcmd.error("printcore: troca já em curso")
        self._check_idle(gcmd)
        h = self.heater
        if h.target_temp > 0.:
            raise gcmd.error("printcore: aquecedor com alvo %.0f C; faz "
                             "M104 S0 e espera arrefecer" % (h.target_temp,))
        temp = max(h.last_temp, h.smoothed_temp)
        if temp > self.safe_temp:
            raise gcmd.error("printcore: hotend a %.0f C; só se troca abaixo "
                             "de %.0f C" % (temp, self.safe_temp))
        # Posição de troca ANTES de bloquear o movimento (se já lá estiver,
        # não mexe). Falha aqui = troca não começa, nada foi alterado.
        self._goto_swap_pos(gcmd)
        # O PWM tem de estar a zero antes de parar as leituras
        deadline = self.reactor.monotonic() + 3.
        while h.last_pwm_value > 0.:
            if self.reactor.monotonic() > deadline:
                raise gcmd.error("printcore: PWM do aquecedor não chegou a "
                                 "zero")
            self._pause(0.2)

        self.swap_active = True
        self._stop_polling()
        # Confirma que as leituras pararam: nenhuma leitura nova durante 1 s,
        # depois de dar tempo à última em trânsito.
        self._pause(1.2)
        seen = h.last_temp_time
        self._pause(1.0)
        if h.last_temp_time != seen:
            self._start_polling()
            self.swap_active = False
            raise gcmd.error("printcore: não consegui confirmar a paragem "
                             "das leituras; troca cancelada, proteção normal "
                             "mantida")
        if self.swap_timeout:
            self.reactor.update_timer(
                self.swap_timer, self.reactor.monotonic() + self.swap_timeout)
        gcmd.respond_info(
            "Modo de troca ativo (aquecimento e movimento bloqueados). Troca "
            "o print core e corre PRINTCORE_SWAP_END%s."
            % (" em menos de %.0f s" % self.swap_timeout
               if self.swap_timeout else ""))

    def cmd_SWAP_END(self, gcmd):
        if not self.swap_active:
            raise gcmd.error("printcore: não há troca em curso")
        want = gcmd.get('NOZZLE', None)
        # 1) ID válido e estável
        if self.state != PRESENT:
            raise gcmd.error("printcore: sem ID de print core válido (estado="
                             "%s); troca continua ativa" % (self.state,))
        if want is not None and want != self.nozzle:
            raise gcmd.error("printcore: montado '%s', esperado '%s'"
                             % (self.nozzle, want))
        # 2) leituras diretas do RTD com falhas limpas
        s = self.sensor
        s.spi.spi_send(s.config_reg)        # limpa falhas "latched"
        self._pause(0.5)
        temps = []
        for _ in range(self.swap_samples):
            value, fault = self._read_rtd()
            if fault:
                raise gcmd.error(
                    "printcore: MAX31865 com falha 0x%02x (contactos do "
                    "RTD?); troca continua ativa" % (fault,))
            try:
                temp = s.calc_temp(value)
            except ValueError:
                raise gcmd.error("printcore: leitura do RTD sem sentido; "
                                 "troca continua ativa")
            if not (self.swap_min_temp <= temp <= self.safe_temp):
                raise gcmd.error(
                    "printcore: RTD lê %.1f C, fora de [%.0f, %.0f]; troca "
                    "continua ativa" % (temp, self.swap_min_temp,
                                        self.safe_temp))
            temps.append(temp)
            self._pause(0.15)
        if max(temps) - min(temps) > self.swap_max_spread:
            raise gcmd.error("printcore: leituras do RTD instáveis (%.1f C "
                             "de dispersão); contacto intermitente? troca "
                             "continua ativa" % (max(temps) - min(temps),))
        avg = sum(temps) / len(temps)
        # 3) rearma a proteção do MCU e confirma que chegam leituras
        before = self.heater.last_temp_time
        self._start_polling()
        deadline = self.reactor.monotonic() + 3.
        while self.heater.last_temp_time == before:
            if self.reactor.monotonic() > deadline:
                self.fault("sem leituras do MAX31865 depois de rearmar a "
                           "proteção")
                return
            self._pause(0.2)
        if abs(self.heater.last_temp - avg) > 5.:
            self.fault("leitura após rearmar (%.1f C) diverge do teste "
                       "direto (%.1f C)" % (self.heater.last_temp, avg))
            return
        self.swap_active = False
        self.reactor.update_timer(self.swap_timer, self.reactor.NEVER)
        msg = ""
        if self.rehome_after_swap:
            # O nozzle novo pode ter outra altura: a referência de Z deixa de
            # valer. Os motores continuam energizados, só se perde o "homed".
            self.printer.lookup_object('toolhead').get_kinematics() \
                .clear_homing_state("xyz")
            msg = " Faz home (G28) antes de mover."
        gcmd.respond_info("Troca concluída: %s, hotend a %.1f C. Proteção "
                          "normal reposta.%s" % (self.nozzle, avg, msg))

    def _swap_timeout_cb(self, eventtime):
        if self.swap_active:
            self.fault("troca do print core não concluída a tempo")
        return self.reactor.NEVER

    # ------------------------------------------------------------------
    def cmd_STATUS(self, gcmd):
        gcmd.respond_info(
            "printcore: estado=%s nozzle=%s adc=%s troca_ativa=%s "
            "aquecimento_permitido=%s movimento_bloqueado=%s"
            % (self.state, self.nozzle or "-",
               "-" if self.counts is None else "%.0f" % self.counts,
               self.swap_active, self.heating_allowed(), self.swap_active))

    def cmd_REQUIRE(self, gcmd):
        want = gcmd.get('NOZZLE', None)
        if self.swap_active:
            raise gcmd.error("printcore: troca em curso")
        if self.state != PRESENT:
            raise gcmd.error("printcore: sem print core válido (estado=%s)"
                             % (self.state,))
        if want is not None and want != self.nozzle:
            raise gcmd.error("printcore: montado '%s', esperado '%s'"
                             % (self.nozzle, want))

    def get_status(self, eventtime):
        return {
            'state': self.state,
            'present': self.state == PRESENT,
            'nozzle': self.nozzle,
            'id_counts': self.counts if self.counts is not None else 0.,
            'swap_active': self.swap_active,
            'motion_locked': self.swap_active,
            'heating_allowed': self.heating_allowed(),
        }


def load_config(config):
    return PrintCore(config)
