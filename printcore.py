# Klipper extras: print core amovível, com MAX31865 (PT1000) e ID por resistência
#
# Princípio: a proteção normal do Klipper fica SEMPRE ativa. Só é aliviada
# dentro de um "modo de troca" explícito, e só se repõe quando a troca é
# validada.
#
#   Fora do modo de troca : comportamento 100% stock. Se o print core for
#                           retirado, o RTD abre e o MCU faz shutdown
#                           ("Thermocouple reader fault"), como em qualquer
#                           Klipper.
#   PRINTCORE_GOTO_SWAP   : [X= Y= Z= SPEED= Z_SPEED=] leva a cabeça à posição
#                           de troca. Sem parâmetros usa a tabela da macro
#                           [gcode_macro PRINT_CORE_POSITIONS].
#   PRINTCORE_SWAP_START  : (mesmos parâmetros) só aceita sem impressão, com XYZ
#                           em home, aquecedor desligado, PWM a zero e hotend
#                           frio. Vai à posição de troca, pára as leituras do
#                           MAX31865 no MCU (é isto que evita o shutdown),
#                           bloqueia o aquecimento E O MOVIMENTO. Confirma que
#                           as leituras pararam.
#   PRINTCORE_SWAP_END    : exige ID de nozzle válido (ou bypass), limpa as
#                           falhas do MAX31865, faz leituras de teste diretas
#                           por SPI (sem falha, temperatura plausível, estável)
#                           e só então rearma as leituras do MCU. Confirma que
#                           chegam leituras. Qualquer falha = shutdown.
#   Timeout               : se a troca não terminar a tempo, shutdown.
#
# Linha de ID com problema (cabo, contacto):
#   - Mudanças de ID com o aquecedor ligado NÃO fazem shutdown enquanto o RTD
#     continuar a ler bem. Só faz shutdown se o RTD deixar de ler.
#   - Se o ID diz "ausente/desconhecido" mas o RTD lê bem durante
#     id_fault_secs, é uma contradição (o RTD só lê com o core montado):
#     o módulo avisa e permite PRINTCORE_ID_BYPASS NOZZLE=x CONFIRM=1.
#   - O bypass vale só até ao próximo reinício (nada é assumido ao arrancar)
#     e desfaz-se sozinho quando o ID volta a ler bem. Mantém todas as proteções do RTD.
#
# Mensagens: log_level 0 (utilizador) ou 1 (developer). Em runtime:
# PRINTCORE_LOG LEVEL=0|1.
#
# Instalação: copiar para klipper/klippy/extras/printcore.py e reiniciar.
# Só funciona com sensor_type: MAX31865 no aquecedor. NÃO TESTADO EM HARDWARE.

import inspect
import logging
import re
from . import spi_temperature

REPORT_TIME = 0.300
ADC_SAMPLE_TIME = 0.001
ADC_SAMPLE_COUNT = 8

ABSENT = "absent"      # ID em aberto
UNKNOWN = "unknown"    # algo ligado, mas o ID não corresponde a nenhum nozzle
PRESENT = "present"    # print core reconhecido

BYPASS_VAR = "printcore_id_bypass"   # chave em [save_variables] (minúsculas)
RTD_STALE = 2.0        # s sem leitura nova = RTD "não lê"
ID_MSG_INTERVAL = 10.  # janela (s) do limite de mensagens de ID (nível 0)
ERR_DEDUP_SECS = 0.5   # o Klipper repete o erro de uma macro em cada nível de aninhamento
SWAP_WARN_SECS = 60.    # aviso antes de o tempo da troca acabar
ID_MSG_BURST = 4       # máx. de mensagens de ID por janela, contra cabos a oscilar

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
    # Happy Hare: ninguém deve carregar filamento nem mexer no seletor com o
    # print core fora (T0..T15 são as macros de troca de ferramenta)
    "MMU_LOAD", "MMU_PRELOAD", "MMU_CHANGE_TOOL", "MMU_SELECT",
    "MMU_SELECT_BYPASS", "MMU_CHECK_GATE",
) + tuple("T%d" % _i for _i in range(16))

QUERY_FMT = ("query_thermocouple oid=%c clock=%u rest_ticks=%u"
             " min_value=%u max_value=%u max_invalid_count=%c")

RTD_MSB_REG = 0x01
FAULT_STATUS_REG = 0x07


def pretty(name):
    # "0.4_standard" -> "0.4 mm standard"
    m = re.match(r'^(\d+(?:\.\d+)?)_(.+)$', name)
    if m:
        return "%s mm %s" % (m.group(1), m.group(2).replace('_', ' '))
    return name.replace('_', ' ')


class PrintCore:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        # Quando um comando dentro de uma macro falha, o Klipper escreve o
        # mesmo erro uma vez por nível de macro (PRINT_CORE_CHANGE_DONE ->
        # PRINTCORE_SWAP_END = 2 vezes). Erros iguais seguidos, em menos de
        # ERR_DEDUP_SECS, escrevem-se só uma vez.
        self._last_err = ("", -1e9)
        _orig_respond_error = getattr(self.gcode, '_respond_error', None)
        if _orig_respond_error is not None:
            def respond_error(msg, _orig=_orig_respond_error):
                now = self.printer.get_reactor().monotonic()
                last_msg, last_t = self._last_err
                self._last_err = (msg, now)
                if msg == last_msg and now - last_t < ERR_DEDUP_SECS:
                    return None
                return _orig(msg)
            self.gcode._respond_error = respond_error
        self.heater_name = config.get('heater', 'extruder')
        self.heater = None
        self.sensor = None
        self.mcu = None
        self.query_cmd = None
        self._faulted = False
        self.log_level = config.getint('log_level', 0, minval=0, maxval=1)
        self._id_msg_times = []

        # --- troca ---
        self.safe_temp = config.getfloat('safe_temp', 50., above=0.)
        # Limite inferior próprio: o min_temp do extruder pode ser muito baixo
        # (ex.: -273) e não serve para validar a leitura no fim da troca.
        self.swap_min_temp = config.getfloat('swap_min_temp', 0.)
        self.swap_timeout = config.getfloat('swap_timeout', 1200., minval=0.)
        self.swap_samples = config.getint('swap_samples', 5, minval=2)
        self.swap_max_spread = config.getfloat('swap_max_spread', 2., above=0.)
        # Depois de trocar, o nozzle pode ter outra altura: obriga a novo home
        self.rehome_after_swap = config.getboolean('rehome_after_swap', True)
        extra = config.get('swap_block_extra', '')
        self.block_cmds = list(BLOCKED_COMMANDS) + [
            c.strip().upper() for c in extra.replace('\n', ',').split(',')
            if c.strip()]
        self.swap_active = False
        self._swap_deadline = 0.
        self._swap_warned = False
        self.swap_timer = self.reactor.register_timer(
            self._swap_timeout_cb, self.reactor.NEVER)
        # Teste do RTD durante a troca (só avisa; quem decide é o SWAP_END)
        self._probe_timer = self.reactor.register_timer(
            self._probe_cb, self.reactor.NEVER)
        self._probing = False
        self._ending = False
        self._rtd_present = False   # o RTD respondia da última vez que se testou
        self._probe_avg = 0.

        # --- ID do nozzle: leitura do ADC em "counts" de 12 bits (0-4096),
        # a mesma escala da tabela de códigos (V / (3.3 / 4096)) ---
        self.tol = config.getfloat('id_tolerance', 15., above=0.)
        self.absent_counts = config.getfloat('absent_counts', 3800.,
                                             above=0., maxval=4096.)
        self.debounce = config.getint('debounce_samples', 3, minval=1)
        self.id_fault_secs = config.getint('id_fault_secs', 5, minval=2)
        # Se definido, ligar o bypass do ID exige CODE=<valor> (só técnico)
        self.bypass_code = config.get('bypass_code', '').strip()
        self.nozzles = self._parse_nozzles(config)
        self._hold = self._hold_windows()
        self.state = ABSENT           # estado lido no pino de ID
        self.nozzle = ""
        self.counts = None
        self._cand = None
        self._cand_count = 0

        # --- linha de ID com problema ---
        self.bypass_nozzle = ""       # "" = sem bypass
        self._started = False         # relatório de arranque já feito
        self.id_fault_suspected = False
        self._contra = 0              # s seguidos de "ID mau e RTD bom"
        self._recover = 0             # s seguidos de "ID bom" com bypass
        self._clearing = False
        self._sv = None
        self.monitor_timer = self.reactor.register_timer(
            self._monitor_cb, self.reactor.NEVER)
        self._startup_timer = self.reactor.register_timer(
            self._startup_cb, self.reactor.NEVER)

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
                 "Print core status"),
                ("PRINTCORE_REQUIRE", self.cmd_REQUIRE,
                 "Fails if the print core/nozzle is not the expected one"),
                ("PRINTCORE_GOTO_SWAP", self.cmd_GOTO_SWAP,
                 "Move the toolhead to the swap position (no locking)"),
                ("PRINTCORE_SWAP_START", self.cmd_SWAP_START,
                 "Start a print core swap (relaxes protection)"),
                ("PRINTCORE_SWAP_EXTEND", self.cmd_SWAP_EXTEND,
                 "Add time to the swap: SECONDS=<30-3600> (default "
                 "swap_timeout)"),
                ("PRINTCORE_SWAP_END", self.cmd_SWAP_END,
                 "End the swap and restore normal protection"),
                ("PRINTCORE_CHECK_SENSOR", self.cmd_CHECK_SENSOR,
                 "In swap mode: test the temperature sensor of the new "
                 "print core"),
                ("PRINTCORE_ID_BYPASS", self.cmd_ID_BYPASS,
                 "Bypass a faulty ID signal: NOZZLE=<name> CONFIRM=1 | "
                 "CLEAR=1"),
                ("PRINTCORE_LOG", self.cmd_LOG,
                 "Message detail: LEVEL=0 (simple) or 1 (detailed)")):
            self.gcode.register_command(name, func, desc=desc)

    # ------------------------------------------------------------------
    # Mensagens: nível 0 = utilizador, nível 1 = developer
    def M(self, user, dev=None):
        if self.log_level >= 1 and dev is not None:
            return dev
        return user

    def _say(self, gcmd, user, dev=None):
        msg = self.M(user, dev)
        if gcmd is not None:
            gcmd.respond_info(msg)
        else:
            self.gcode.respond_info(msg)

    def _code_ok(self, gcmd):
        return (not self.bypass_code
                or gcmd.get('CODE', '') == self.bypass_code)

    def _code_error(self, gcmd):
        return gcmd.error(self.M(
            "Print core [PC-09]: a service code is needed to continue "
            "without the ID. Contact support. The swap stays active.",
            "printcore: bypass_code set; CODE=<code> missing or wrong"))

    def _id_say_ok(self):
        # Contra oscilações de um cabo mau: no nível 0 no máx. ID_MSG_BURST
        # mensagens por ID_MSG_INTERVAL s. Durante uma troca (deliberada) diz
        # sempre: o utilizador tira e põe o core em poucos segundos e tem de
        # ver as duas mensagens.
        if self.log_level >= 1 or self.swap_active:
            return True
        now = self.reactor.monotonic()
        self._id_msg_times = [t for t in self._id_msg_times
                              if now - t < ID_MSG_INTERVAL]
        if len(self._id_msg_times) < ID_MSG_BURST:
            self._id_msg_times.append(now)
            return True
        return False

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
                raise config.error("printcore: invalid line in nozzles: "
                                   "'%s' (expected 'name: counts')" % (line,))
        if not nozzles:
            raise config.error("printcore: nozzles is empty")
        nozzles.sort(key=lambda n: n[1])
        # As janelas [counts - tol, counts + tol] não podem sobrepor-se
        for (na, ca), (nb, cb) in zip(nozzles, nozzles[1:]):
            if ca + self.tol >= cb - self.tol:
                raise config.error(
                    "printcore: IDs '%s' (%.0f) and '%s' (%.0f) overlap "
                    "with id_tolerance=%.0f" % (na, ca, nb, cb, self.tol))
        top_name, top = nozzles[-1]
        if top + self.tol >= self.absent_counts:
            raise config.error(
                "printcore: '%s' (%.0f) too close to absent_counts "
                "(%.0f)" % (top_name, top, self.absent_counts))
        return nozzles

    def _hold_windows(self):
        # Histerese: um nozzle já reconhecido mantém-se numa janela maior
        # (1.5 x id_tolerance), para a deriva do ID com a temperatura não o
        # fazer "desaparecer". Nunca invade a janela estrita do vizinho nem
        # a zona de "ausente".
        hold = {}
        for i, (name, c) in enumerate(self.nozzles):
            h = self.tol * 1.5
            if i > 0:
                h = min(h, c - self.nozzles[i - 1][1] - self.tol - 1.)
            if i < len(self.nozzles) - 1:
                h = min(h, self.nozzles[i + 1][1] - c - self.tol - 1.)
            h = min(h, self.absent_counts - c - 1.)
            hold[name] = (c, max(h, self.tol))
        return hold

    def _nozzle_names(self):
        return [n for n, _ in self.nozzles]

    def _handle_connect(self):
        pheaters = self.printer.lookup_object('heaters')
        self.heater = pheaters.lookup_heater(self.heater_name)
        sensor = self.heater.sensor
        if not isinstance(sensor, spi_temperature.MAX31865):
            raise self.printer.config_error(
                "printcore: heater '%s' must use sensor_type: "
                "MAX31865" % (self.heater_name,))
        self.sensor = sensor
        self.mcu = sensor.mcu
        self.query_cmd = self.mcu.lookup_command(QUERY_FMT)

        # Bloqueia pedidos de temperatura > 0 em modo de troca ou sem ID
        # válido (cobre M104, M109, SET_HEATER_TEMPERATURE, macros...)
        orig_set_temp = self.heater.set_temp

        def guarded_set_temp(degrees):
            if not self._temp_change_ok(degrees):
                raise self.printer.command_error(self._heating_blocked_msg())
            return orig_set_temp(degrees)

        self.heater.set_temp = guarded_set_temp

        # alter_target (usado pelo PID_CALIBRATE) também muda o alvo
        orig_alter_target = self.heater.alter_target

        def guarded_alter_target(target_temp):
            if not self._temp_change_ok(target_temp):
                # Corre dentro do callback de leituras: levantar exceção aqui
                # pararia o PWM e só daria um shutdown obscuro. Desliga o
                # aquecedor, que é sempre seguro.
                logging.warning("printcore: heating blocked in alter_target, "
                                "switching the heater off")
                return orig_alter_target(0.)
            return orig_alter_target(target_temp)

        self.heater.alter_target = guarded_alter_target

    def _temp_change_ok(self, degrees):
        if degrees <= 0. or self.heating_allowed():
            return True
        # Aquecedor já ligado (impressão em curso) e o RTD lê bem: o ID pode
        # ter oscilado, mas o que protege é o RTD. Deixa seguir as mudanças
        # de temperatura (M104/M109 na camada 2, etc.) em vez de abortar a
        # impressão. Ligar de novo continua bloqueado sem ID válido.
        return (not self.swap_active and not self._faulted
                and self.heating() and self._rtd_ok())

    def _heating_blocked_msg(self):
        return self.M(
            "Print core: heating is disabled (no valid print core, or a "
            "swap is in progress).",
            "printcore: heating blocked (swap_active=%s, id_state=%s, "
            "eff_state=%s)" % (self.swap_active, self.state, self.eff_state))

    # ------------------------------------------------------------------
    # Estado efetivo: o ID lido, ou o bypass quando o ID não lê
    def _bypass_active(self):
        return bool(self.bypass_nozzle) and self.state != PRESENT

    @property
    def eff_state(self):
        if self.state == PRESENT or self._bypass_active():
            return PRESENT
        return self.state

    @property
    def eff_nozzle(self):
        if self.state == PRESENT:
            return self.nozzle
        if self._bypass_active():
            return self.bypass_nozzle
        return ""

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
        logging.info("printcore: commands blocked during swap: %s",
                     ", ".join(wrapped))
        # 2.ª barreira: qualquer movimento que chegue ao toolhead (macros,
        # transforms, extrusor, homing). Atributos de instância por cima dos
        # métodos; quem chama self.move()/self.drip_move() passa aqui.
        toolhead = self.printer.lookup_object('toolhead')
        orig_move = toolhead.move
        for attr in ('move', 'drip_move', 'manual_move'):
            setattr(toolhead, attr, self._make_guard(getattr(toolhead, attr)))
        # O gcode_move guardou uma referência ao toolhead.move original (no seu
        # klippy:ready, que corre antes): sem isto, SET_GCODE_OFFSET MOVE=1 e
        # RESTORE_GCODE_STATE MOVE=1 passariam por cima do guard. Só se troca
        # se ainda for a original (com bed_mesh o transform já chama o
        # toolhead.move pelo nome e passa no guard).
        gm = self.printer.lookup_object('gcode_move', None)
        if gm is not None and getattr(gm, 'move_with_transform', None) \
                == orig_move:
            gm.move_with_transform = toolhead.move
        # Bypass guardado de uma sessão anterior
        self._load_bypass()
        self.reactor.update_timer(self.monitor_timer,
                                  self.reactor.monotonic() + 1.)
        self.reactor.update_timer(self._startup_timer,
                                  self.reactor.monotonic() + 6.)

    def _motion_msg(self, name=None):
        return self.M(
            "Print core: movement and filament loading are disabled during "
            "the print core swap. Mount the print core and run "
            "PRINT_CORE_CHANGE_DONE.",
            "printcore: command blocked - print core swap in progress. Mount "
            "the print core and run PRINT_CORE_CHANGE_DONE%s"
            % (" (%s)" % name if name else ""))

    def _make_block(self, name, old):
        def blocked(gcmd):
            if self.swap_active:
                raise gcmd.error(self._motion_msg(name))
            return old(gcmd)
        return blocked

    def _make_guard(self, orig):
        def guard(*args, **kwargs):
            if self.swap_active:
                raise self.printer.command_error(self._motion_msg())
            return orig(*args, **kwargs)
        return guard

    def _check_idle(self, gcmd):
        ps = self.printer.lookup_object('print_stats', None)
        if ps is not None:
            state = ps.get_status(self.reactor.monotonic()).get('state')
            if state in ('printing', 'paused'):
                raise gcmd.error(self.M(
                    "Print core: a print is in progress. Cancel it before "
                    "swapping.",
                    "printcore: print in progress (%s); cancel it before "
                    "swapping" % state))

    def _swap_params(self, gcmd):
        # Fonte única: tabela [gcode_macro PRINT_CORE_POSITIONS]. Parâmetros
        # no comando (X Y Z SPEED Z_SPEED) têm prioridade sobre a tabela.
        # Coordenadas da máquina.
        tbl = {}
        macro = self.printer.lookup_object(
            'gcode_macro PRINT_CORE_POSITIONS', None)
        if macro is not None:
            tbl = getattr(macro, 'variables', {}) or {}
        generic = ("Print core: the swap position is not set up correctly. "
                   "Check PRINT_CORE_POSITIONS.")

        def pick(param, var, default=None):
            if gcmd.get(param, None) is not None:
                return None  # vem do comando
            if var in tbl:
                try:
                    return float(tbl[var])
                except (TypeError, ValueError):
                    raise gcmd.error(self.M(
                        generic,
                        "printcore: %s in PRINT_CORE_POSITIONS is not a "
                        "number: %r" % (var, tbl[var])))
            if default is not None:
                return default
            raise gcmd.error(self.M(
                generic,
                "printcore: missing %s: pass it in the command or define %s "
                "in [gcode_macro PRINT_CORE_POSITIONS]" % (param, var)))

        def val(param, var, default=None, **kw):
            t = pick(param, var, default)
            if t is None:
                return gcmd.get_float(param, **kw)
            lo, above = kw.get('minval'), kw.get('above')
            if (lo is not None and t < lo) or (above is not None
                                               and t <= above):
                raise gcmd.error(self.M(
                    generic,
                    "printcore: %s=%s invalid in PRINT_CORE_POSITIONS"
                    % (var, t)))
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
            raise gcmd.error(self.M(
                "Print core: home the printer first.",
                "printcore: home XYZ before moving to the swap position "
                "(homed='%s')" % (homed,)))
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
            raise gcmd.error(self.M(
                "Print core: the toolhead did not reach the swap position.",
                "printcore: the toolhead did not reach the swap position "
                "(%.2f, %.2f, %.2f)" % tuple(pos[:3])))
        return x, y, z

    def cmd_GOTO_SWAP(self, gcmd):
        if self.swap_active:
            raise gcmd.error(self.M("Print core: a swap is already in "
                                    "progress.",
                                    "printcore: swap already in progress"))
        self._check_idle(gcmd)
        x, y, z = self._goto_swap_pos(gcmd)
        self._say(gcmd, "Print core: toolhead is in the swap position.",
                  "printcore: toolhead at swap position X%.1f Y%.1f Z%.1f"
                  % (x, y, z))

    # ------------------------------------------------------------------
    def heating(self):
        return self.heater is not None and self.heater.target_temp > 0.

    def heating_allowed(self):
        return (not self.swap_active and self.eff_state == PRESENT
                and not self._faulted)

    def fault(self, user, dev=None):
        if self._faulted:
            return
        self._faulted = True
        logging.error("printcore: %s", dev if dev is not None else user)
        self.printer.invoke_shutdown(self.M(user, dev))

    # ------------------------------------------------------------------
    # O RTD lê? (leitura recente e plausível, com as leituras do MCU ativas)
    def _rtd_ok(self):
        h = self.heater
        if h is None or self.mcu is None or self.swap_active \
                or self._faulted:
            return False
        now = self.mcu.estimated_print_time(self.reactor.monotonic())
        if now - h.last_temp_time > RTD_STALE:
            return False
        return -50. <= h.last_temp <= getattr(h, 'max_temp', 400.)

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
        if self.state == PRESENT and self.nozzle in self._hold:
            c, h = self._hold[self.nozzle]
            if abs(counts - c) <= h:
                return PRESENT, self.nozzle
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
                "printcore: this Klipper has no known ADC API")
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

    def _id_callback(self, *args):
        # Klipper atual: callback(samples), samples = [(tempo, valor), ...].
        # Versões antigas: callback(tempo, valor).
        samples = args[0] if len(args) == 1 else [args]
        for sample in samples:
            try:
                _, read_value = sample
            except (TypeError, ValueError):
                continue
            self._id_sample(read_value)

    def _id_sample(self, read_value):
        counts = read_value * 4096.
        self.counts = counts          # sempre a leitura mais recente
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
        if self.swap_active:
            if state != PRESENT:
                self._rtd_present = False   # reavaliar o RTD no próximo teste
            # Print core novo montado: testa o sensor de temperatura logo,
            # em vez de esperar pelo fim da troca.
            self.reactor.update_timer(
                self._probe_timer,
                self.reactor.monotonic() + (1.0 if state == PRESENT
                                            else 3.0))
        if self.heating():
            # Já não há shutdown só por o ID mudar: o que protege é o RTD.
            if self._rtd_ok():
                if self._id_say_ok():
                    self._say(
                        None,
                        "Print core: the ID signal changed while heating. "
                        "The temperature sensor is fine, so the printer "
                        "keeps going.",
                        "printcore: ID changed from '%s/%s' to '%s/%s' while "
                        "heating; RTD ok, continuing" % (prev + (state,
                                                                 nozzle)))
            else:
                self.fault(
                    "Print core: the ID changed and the temperature sensor "
                    "is not responding. Printer stopped.",
                    "printcore: ID changed from '%s/%s' to '%s/%s' with "
                    "heater active and RTD not readable"
                    % (prev + (state, nozzle)))
            return
        if not self._started:
            # Antes do relatório de arranque nada é dito: é o _startup_cb
            # que o diz, uma só vez, depois de o Happy Hare e o resto
            # terem escrito as suas mensagens de arranque.
            logging.info("printcore: ID %s/%s before the startup report "
                         "(ADC %s)", state, nozzle, counts)
            return
        if not self._id_say_ok():
            return
        if state == PRESENT:
            self._say(None, "Print core: %s nozzle detected." % pretty(nozzle),
                      "printcore: detected %s (ADC %.0f)" % (nozzle, counts))
        elif state == ABSENT:
            self._say(None, "Print core: removed.",
                      "printcore: removed (ADC %.0f)" % counts)
        else:
            self._say(None,
                      "!! Print core: not recognized. Heating is disabled.",
                      "!! printcore: unknown ID (ADC %.0f), heating blocked"
                      % counts)

    # ------------------------------------------------------------------
    # Linha de ID com problema: deteção, bypass e recuperação
    def _load_bypass(self):
        self._sv = self.printer.lookup_object('save_variables', None)
        if self._sv is None:
            logging.warning("printcore: no [save_variables]; ID bypass "
                            "will not survive a restart")
            return
        # O bypass NÃO sobrevive a um reinício: ao arrancar nada é assumido.
        # Se havia um guardado, apaga-se e o arranque pede o ID de novo.
        name = self._sv.allVariables.get(BYPASS_VAR, "")
        if name:
            logging.warning("printcore: discarding saved ID bypass '%s' "
                            "(nothing is assumed after a restart)", name)
            self._save_bypass(None, "")

    def _save_bypass(self, gcmd, name):
        # Devolve True se ficou guardado em [save_variables]
        if self._sv is None:
            return False
        cmd = "SAVE_VARIABLE VARIABLE=%s VALUE=\"'%s'\"" % (BYPASS_VAR, name)
        try:
            if gcmd is not None:
                self.gcode.run_script_from_command(cmd)
            else:
                self.gcode.run_script(cmd)
            return True
        except Exception:
            logging.exception("printcore: could not save the ID bypass")
            return False

    def _id_hint(self):
        # Hipóteses prováveis conforme o que o ADC lê (só para o nível 1)
        c = self.counts
        if self.state == ABSENT:
            return ("open ID circuit (ADC ~4095): cable/connector open or "
                    "intermittent, ID resistor missing/desoldered on the "
                    "print core PCB, bad contact on the toolhead port, or "
                    "no pull-up on TH0")
        if c is not None and c < 100.:
            return ("ID shorted to ground (ADC ~0): damaged cable, solder "
                    "bridge on the print core PCB, or short on the toolhead "
                    "port (also the unlisted code 1111)")
        return ("ID outside every table window (ADC %s): wrong or "
                "damaged ID resistor, poor contact (partial value), "
                "unlisted code, or noise on TH0"
                % ("-" if c is None else "%.0f" % c))

    def _monitor_cb(self, eventtime):
        # Corre a cada 1 s. NUNCA pode levantar exceção (o reactor não a
        # apanha e o Klipper cairia).
        try:
            ok = self._rtd_ok()
            if not ok or self.state == PRESENT:
                self._contra = 0
            else:
                self._contra += 1
            was = self.id_fault_suspected
            self.id_fault_suspected = self._contra >= self.id_fault_secs
            if self.id_fault_suspected and not was \
                    and not self._bypass_active():
                names = ", ".join(self._nozzle_names())
                if self.bypass_code:
                    user = ("Print core: inserted, but its ID cannot be "
                            "read, so the nozzle size is unknown. Check the "
                            "print core connection. If it keeps happening, "
                            "contact support.")
                else:
                    user = ("Print core: inserted, but its ID cannot be "
                            "read, so the nozzle size is unknown. Check the "
                            "print core connection. To keep printing, run "
                            "PRINTCORE_ID_BYPASS NOZZLE=<name> CONFIRM=1 "
                            "(names: %s)" % names)
                self._say(
                    None,
                    user,
                    "printcore: ID fault suspected (id_state=%s, ADC %s), "
                    "RTD ok at %.1f C. Possible: %s. Run "
                    "PRINTCORE_ID_BYPASS NOZZLE=<name> CONFIRM=1 (names: %s)"
                    % (self.state, "-" if self.counts is None
                       else "%.0f" % self.counts, self.heater.last_temp,
                       self._id_hint(), names))
            # Bypass ativo e o ID voltou a ler bem de forma estável
            if self.bypass_nozzle and self.state == PRESENT and ok:
                self._recover += 1
            else:
                self._recover = 0
            if self._recover >= self.id_fault_secs and not self._clearing:
                self._clearing = True
                self.reactor.register_async_callback(self._auto_clear_cb)
        except Exception:
            logging.exception("printcore: monitor error")
        return eventtime + 1.

    def _startup_cb(self, eventtime):
        # Uma vez, pouco depois de arrancar, diz o que há montado. A primeira
        # leitura do ID acontece ANTES de o Klipper estar pronto, quando o
        # Moonraker ainda não está ligado, e essa mensagem perdia-se.
        self._started = True
        try:
            if self.swap_active or self._faulted:
                return self.reactor.NEVER
            if self._bypass_active():
                self._say(None,
                          "Print core: the ID cannot be read. The ID bypass "
                          "is on (%s nozzle assumed)."
                          % pretty(self.bypass_nozzle),
                          "printcore: startup: ID bypass active (%s)"
                          % self.bypass_nozzle)
            elif self.state == PRESENT:
                self._say(None,
                          "Print core: %s nozzle detected."
                          % pretty(self.nozzle),
                          "printcore: startup: detected %s (ADC %.0f)"
                          % (self.nozzle, self.counts or 0.))
            elif self._rtd_ok():
                # Core montado e ID sem ler: a mensagem sai no próximo
                # segundo, sem esperar o tempo de suspeita (que serve para
                # não alarmar durante uma inserção; no arranque não há).
                logging.info("printcore: startup: ID %s, RTD ok -> ID fault",
                             self.state)
                self._contra = self.id_fault_secs
            else:
                self._say(None,
                          "Print core: no print core detected.",
                          "printcore: startup: no print core (ID %s, RTD "
                          "not readable)" % (self.state,))
        except Exception:
            logging.exception("printcore: startup report error")
        return self.reactor.NEVER

    def _auto_clear_cb(self, eventtime):
        try:
            self.bypass_nozzle = ""
            self._save_bypass(None, "")
            self._say(None, "Print core: the ID signal recovered. Bypass "
                      "removed.",
                      "printcore: ID signal recovered (%s), bypass removed"
                      % self.nozzle)
        except Exception:
            logging.exception("printcore: auto-clear error")
        finally:
            self._clearing = False
            self._recover = 0

    def cmd_ID_BYPASS(self, gcmd):
        if gcmd.get_int('CLEAR', 0):
            had = self.bypass_nozzle
            self.bypass_nozzle = ""
            saved = self._save_bypass(gcmd, "")
            if not had:
                self._say(gcmd, "Print core: the ID bypass was not active.",
                          "printcore: ID bypass was not active")
            else:
                self._say(gcmd, "Print core: ID bypass removed.",
                          "printcore: ID bypass removed (saved=%s)" % saved)
            return
        name = gcmd.get('NOZZLE', None)
        if name is None:
            names = ", ".join(self._nozzle_names())
            usage = ("Nozzles: %s. Use: PRINTCORE_ID_BYPASS NOZZLE=<name> "
                     "CONFIRM=1" % names)
            if self.bypass_nozzle:
                self._say(gcmd, "Print core: ID bypass is ON (%s nozzle "
                          "assumed). Undo with PRINTCORE_ID_BYPASS CLEAR=1. "
                          "%s" % (pretty(self.bypass_nozzle), usage),
                          "printcore: ID bypass ON (%s). %s"
                          % (self.bypass_nozzle, usage))
            else:
                self._say(gcmd, "Print core: ID bypass is OFF. %s" % usage,
                          "printcore: ID bypass OFF (id_fault_suspected=%s). "
                          "%s" % (self.id_fault_suspected, usage))
            return
        if not self._code_ok(gcmd):
            raise self._code_error(gcmd)
        if gcmd.get_int('CONFIRM', 0) != 1:
            raise gcmd.error(self.M(
                "Print core: add CONFIRM=1 to confirm. You are telling the "
                "printer which print core is mounted.",
                "printcore: CONFIRM=1 required"))
        if self.swap_active:
            raise gcmd.error(self.M(
                "Print core: a swap is in progress.",
                "printcore: swap in progress"))
        if self.heating():
            raise gcmd.error(self.M(
                "Print core: turn the heater off first (M104 S0).",
                "printcore: heater target > 0; run M104 S0 first"))
        self._check_idle(gcmd)
        if name not in self._nozzle_names():
            raise gcmd.error(self.M(
                "Print core: unknown nozzle '%s'. Valid names: %s"
                % (name, ", ".join(self._nozzle_names())),
                "printcore: unknown nozzle '%s' (valid: %s)"
                % (name, ", ".join(self._nozzle_names()))))
        if not self.id_fault_suspected:
            if self.state == PRESENT:
                raise gcmd.error(self.M(
                    "Print core: the ID signal looks fine, so a bypass is "
                    "not needed.",
                    "printcore: ID reads %s; bypass not needed"
                    % self.nozzle))
            raise gcmd.error(self.M(
                "Print core: no ID fault detected yet. The temperature "
                "sensor must read normally for %d s with the print core "
                "mounted. Wait and try again." % self.id_fault_secs,
                "printcore: ID fault not confirmed (RTD ok for %d/%d s)"
                % (self._contra, self.id_fault_secs)))
        self.bypass_nozzle = name
        saved = self._save_bypass(gcmd, name)
        if saved:
            self._say(gcmd,
                      "Print core: ID check bypassed. Assuming %s nozzle. "
                      "Run PRINTCORE_ID_BYPASS CLEAR=1 to undo."
                      % pretty(name),
                      "printcore: ID bypass ON (%s, saved=True)" % name)
        else:
            why = ("no [save_variables]" if self._sv is None
                   else "saving failed, see klippy.log")
            self._say(gcmd,
                      "Print core: ID check bypassed. Assuming %s nozzle. "
                      "Warning: it will be lost on restart (%s)."
                      % (pretty(name), why),
                      "printcore: ID bypass ON (%s, NOT saved: %s)"
                      % (name, why))

    def cmd_LOG(self, gcmd):
        lvl = gcmd.get_int('LEVEL', None, minval=0, maxval=1)
        if lvl is None:
            self._say(gcmd, "Print core: log level is %d (0 = simple, "
                      "1 = detailed)." % self.log_level)
            return
        self.log_level = lvl
        self._say(gcmd, "Print core: log level set to %d (0 = simple, "
                  "1 = detailed). Resets on restart." % lvl)

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
            raise gcmd.error(self.M("Print core: the heater is not "
                                    "available yet.",
                                    "printcore: heater not available yet"))
        if self.swap_active:
            raise gcmd.error(self.M("Print core: a swap is already in "
                                    "progress.",
                                    "printcore: swap already in progress"))
        self._check_idle(gcmd)
        h = self.heater
        if h.target_temp > 0.:
            raise gcmd.error(self.M(
                "Print core: the hotend is heating. Turn it off (M104 S0) "
                "and let it cool down.",
                "printcore: heater target %.0f C; run M104 S0 and wait for "
                "it to cool" % (h.target_temp,)))
        temp = max(h.last_temp, h.smoothed_temp)
        if temp > self.safe_temp:
            raise gcmd.error(self.M(
                "Print core: the hotend is too hot. Let it cool down, then "
                "try again.",
                "printcore: hotend at %.0f C; swap only allowed below %.0f C"
                % (temp, self.safe_temp)))
        # Posição de troca ANTES de bloquear o movimento (se já lá estiver,
        # não mexe). Falha aqui = troca não começa, nada foi alterado.
        self._goto_swap_pos(gcmd)
        # O PWM tem de estar a zero antes de parar as leituras
        deadline = self.reactor.monotonic() + 3.
        while h.last_pwm_value > 0.:
            if self.reactor.monotonic() > deadline:
                raise gcmd.error(self.M(
                    "Print core: the heater did not switch off. Swap "
                    "cancelled.",
                    "printcore: heater PWM did not reach zero"))
            self._pause(0.2)

        rtd_at_start = self._rtd_ok()
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
            raise gcmd.error(self.M(
                "Print core: could not enter swap mode safely. Swap "
                "cancelled; normal protection kept.",
                "printcore: could not confirm sensor polling stopped; swap "
                "cancelled, normal protection kept"))
        if self.swap_timeout:
            self._arm_swap_timer(self.swap_timeout)
        # Com o ID sem ler (bypass), o RTD é o único sinal de que o core foi
        # tirado ou posto: testa-o de 3 em 3 s durante a troca.
        self._rtd_present = rtd_at_start
        if self.state != PRESENT:
            self.reactor.update_timer(self._probe_timer,
                                      self.reactor.monotonic() + 3.0)
        if self.bypass_nozzle:
            # O core vai ser trocado: um bypass antigo não vale para o novo
            logging.info("printcore: swap start clears ID bypass (%s)",
                         self.bypass_nozzle)
            self.bypass_nozzle = ""
            self._save_bypass(gcmd, "")
        self._say(
            gcmd,
            "Print core: swap mode on. Heating and movement are disabled. "
            "Replace the print core, then confirm.",
            "printcore: swap mode active (heating and motion locked). "
            "Replace the print core and run PRINTCORE_SWAP_END%s."
            % (" within %.0f s" % self.swap_timeout
               if self.swap_timeout else ""))

    def _rtd_probe(self):
        # Testa o RTD por SPI (o polling do MCU está parado na troca).
        # Devolve None se estiver bem (e deixa a média em self._probe_avg),
        # ou (msg_utilizador, msg_developer).
        s = self.sensor
        reseat = (" Reseat the print core once. If it happens again, this "
                  "print core is faulty: replace it with another one.")
        s.spi.spi_send(s.config_reg)        # limpa falhas "latched"
        self._pause(0.5)
        temps = []
        for _ in range(self.swap_samples):
            value, fault = self._read_rtd()
            if fault:
                return ("Print core [PC-03]: the temperature sensor of "
                        "this print core does not respond." + reseat,
                        "printcore: MAX31865 fault 0x%02x (RTD contacts?)"
                        % (fault,))
            try:
                temp = s.calc_temp(value)
            except ValueError:
                return ("Print core [PC-04]: the temperature sensor of "
                        "this print core gives an invalid reading." + reseat,
                        "printcore: RTD reading out of range")
            if not (self.swap_min_temp <= temp <= self.safe_temp):
                return ("Print core [PC-05]: the temperature sensor of "
                        "this print core reads an unexpected value." + reseat,
                        "printcore: RTD reads %.1f C, outside [%.0f, %.0f]"
                        % (temp, self.swap_min_temp, self.safe_temp))
            temps.append(temp)
            self._pause(0.15)
        if max(temps) - min(temps) > self.swap_max_spread:
            return ("Print core [PC-06]: the temperature sensor of "
                    "this print core is unstable." + reseat,
                    "printcore: unstable RTD readings (%.1f C spread)"
                    % (max(temps) - min(temps),))
        self._probe_avg = sum(temps) / len(temps)
        return None

    def _probe_cb(self, eventtime):
        # Corre num timer: nunca pode levantar exceção (seria shutdown).
        if not self.swap_active or self._ending or self._probing:
            return self.reactor.NEVER
        present = self.state == PRESENT
        self._probing = True
        try:
            res = self._rtd_probe()
            if present:
                self._rtd_present = res is None
                if res is None:
                    self._say(None, "Print core: temperature sensor OK.",
                              "printcore: swap RTD probe ok")
                else:
                    self._say(None,
                              "!! " + res[0] + " The swap stays active.",
                              "!! " + res[1] + "; swap stays active")
            elif res is None and not self._rtd_present:
                # O RTD passou a responder e o ID não lê: foi posto um core
                self._rtd_present = True
                names = ", ".join(self._nozzle_names())
                self._say(None,
                          "!! Print core [PC-02]: a print core is mounted "
                          "(the temperature sensor responds) but its ID "
                          "cannot be read, so the nozzle size is unknown. "
                          "%s"
                          % ("Reseat it and check the ID contact. If it "
                             "keeps happening, contact support."
                             if self.bypass_code else
                             "Reseat it and check the ID contact, or "
                             "finish with PRINT_CORE_CHANGE_DONE "
                             "NOZZLE=<name> CONFIRM=1 (names: %s)." % names),
                          "!! printcore: RTD ok but ID %s (ADC %.0f)"
                          % (self.state, self.counts or 0.))
            elif res is not None and self._rtd_present:
                # O RTD deixou de responder: o core foi retirado
                self._rtd_present = False
                self._say(None,
                          "Print core: removed (the temperature sensor no "
                          "longer responds).",
                          "printcore: removed (RTD stopped responding, ID "
                          "%s)" % (self.state,))
        except Exception:
            logging.exception("printcore: swap RTD probe failed")
        finally:
            self._probing = False
        if self.swap_active and self.state != PRESENT:
            return self.reactor.monotonic() + 3.0
        return self.reactor.NEVER

    def cmd_CHECK_SENSOR(self, gcmd):
        if not self.swap_active:
            raise gcmd.error(self.M(
                "Print core: only available during a swap.",
                "printcore: PRINTCORE_CHECK_SENSOR needs swap mode"))
        while self._probing:
            self._pause(0.1)
        self._probing = True
        try:
            res = self._rtd_probe()
        finally:
            self._probing = False
        if res is not None:
            raise gcmd.error(self.M(*res))
        self._say(gcmd, "Print core: temperature sensor OK.",
                  "printcore: swap RTD probe ok")

    def cmd_SWAP_END(self, gcmd):
        self._ending = True
        try:
            while self._probing:
                self._pause(0.1)
            self._swap_end(gcmd)
        finally:
            self._ending = False
            if self.swap_active and self.state != PRESENT:
                self.reactor.update_timer(self._probe_timer,
                                          self.reactor.monotonic() + 3.0)

    def _swap_end_no_id(self, gcmd, want):
        # O ID não lê. O RTD diz-nos se há mesmo um print core montado.
        res = self._rtd_probe()
        names = self._nozzle_names()
        if res is not None:
            raise gcmd.error(self.M(
                "Print core [PC-01]: no print core detected. Neither the ID "
                "nor the temperature sensor respond. If one is mounted, "
                "reseat it and check its contacts, then confirm again. The "
                "swap stays active.",
                "printcore: ID %s (ADC %.0f) and RTD not readable (%s); "
                "swap stays active" % (self.state, self.counts, res[1])))
        # O RTD responde: há um print core montado e o que falha é o ID.
        if want in names and gcmd.get_int('CONFIRM', 0) == 1:
            if not self._code_ok(gcmd):
                raise self._code_error(gcmd)
            self.bypass_nozzle = want
            saved = self._save_bypass(gcmd, want)
            self._say(gcmd,
                      "Print core: ID check bypassed. Assuming %s nozzle. "
                      "Run PRINTCORE_ID_BYPASS CLEAR=1 to undo."
                      % pretty(want),
                      "printcore: ID bypass ON at swap end (%s, saved=%s)"
                      % (want, saved))
            return
        lst = ", ".join(names)
        if self.bypass_code:
            user = ("Print core [PC-02]: the print core is mounted (the "
                    "temperature sensor responds) but its ID cannot be "
                    "read. Reseat it; if it still fails, contact support. "
                    "The swap stays active.")
        else:
            user = ("Print core [PC-02]: the print core is mounted (the "
                    "temperature sensor responds) but its ID cannot be "
                    "read. Reseat it and check the ID contact. To continue "
                    "anyway, say which print core it is: "
                    "PRINT_CORE_CHANGE_DONE NOZZLE=<name> CONFIRM=1 "
                    "(names: %s). The swap stays active." % lst)
        raise gcmd.error(self.M(
            user,
            "printcore: RTD ok but ID %s (ADC %.0f); need NOZZLE=<name> "
            "CONFIRM=1%s (valid: %s); swap stays active"
            % (self.state, self.counts,
               " CODE=<bypass_code>" if self.bypass_code else "", lst)))

    def _swap_end(self, gcmd):
        if not self.swap_active:
            raise gcmd.error(self.M("Print core: no swap in progress.",
                                    "printcore: no swap in progress"))
        want = gcmd.get('NOZZLE', None)
        # AUTOHOME=1: a macro PRINT_CORE_CHANGE_DONE faz G28 logo a seguir,
        # por isso a mensagem final não manda fazer home.
        autohome = gcmd.get_int('AUTOHOME', 0) == 1
        # 1) ID válido e estável (ou bypass declarado pelo utilizador)
        if self.eff_state != PRESENT:
            self._swap_end_no_id(gcmd, want)
        if self._bypass_active():
            # O ID não lê: um bypass de antes não pode valer para o print
            # core novo. Quem confirma diz qual montou (pode ser outro).
            names = ", ".join(self._nozzle_names())
            if want is None or want not in self._nozzle_names():
                raise gcmd.error(self.M(
                    "Print core [PC-02]: the ID cannot be read. %s"
                    "The swap stays active."
                    % ("Contact support. " if self.bypass_code else
                       "Say which print core you mounted: "
                       "PRINT_CORE_CHANGE_DONE NOZZLE=<name> (names: %s). "
                       % names),
                    "printcore: ID bypass active, NOZZLE=<name> required at "
                    "swap end (valid: %s); swap stays active" % names))
            if not self._code_ok(gcmd):
                raise self._code_error(gcmd)
            if want != self.bypass_nozzle:
                self.bypass_nozzle = want
                self._save_bypass(gcmd, want)
        if want is not None and want != self.eff_nozzle:
            raise gcmd.error(self.M(
                "Print core [PC-08]: a different print core is mounted than "
                "expected. The swap stays active.",
                "printcore: mounted '%s', expected '%s'"
                % (self.eff_nozzle, want)))
        # 2) leituras diretas do RTD. Um print core com o sensor mau não
        # pode ser usado: recusa, a troca continua ativa e o aviso já foi
        # dado quando o core foi montado.
        res = self._rtd_probe()
        if res is not None:
            raise gcmd.error(self.M(res[0] + " The swap stays active.",
                                    res[1] + "; swap stays active"))
        avg = self._probe_avg
        # 3) rearma a proteção do MCU e confirma que chegam leituras
        before = self.heater.last_temp_time
        self._start_polling()
        deadline = self.reactor.monotonic() + 3.
        while self.heater.last_temp_time == before:
            if self.reactor.monotonic() > deadline:
                self.fault(
                    "Print core: the temperature sensor is not responding "
                    "after the swap. Printer stopped.",
                    "printcore: no MAX31865 readings after re-arming "
                    "protection")
                return
            self._pause(0.2)
        if abs(self.heater.last_temp - avg) > 5.:
            self.fault(
                "Print core: temperature sensor readings are inconsistent. "
                "Printer stopped.",
                "printcore: reading after re-arm (%.1f C) differs from "
                "direct test (%.1f C)" % (self.heater.last_temp, avg))
            return
        self.swap_active = False
        self.reactor.update_timer(self.swap_timer, self.reactor.NEVER)
        user_tail = dev_tail = ""
        if self.rehome_after_swap:
            # O nozzle novo pode ter outra altura: a referência de Z deixa de
            # valer. Os motores continuam energizados, só se perde o "homed".
            self.printer.lookup_object('toolhead').get_kinematics() \
                .clear_homing_state("xyz")
            if autohome:
                user_tail = " Homing now."
                dev_tail = " Homing (G28) next."
            else:
                user_tail = " Home the printer before moving."
                dev_tail = " Home (G28) before moving."
        if self._bypass_active():
            user = ("Print core: swap complete (ID check bypassed, assuming "
                    "%s nozzle).%s" % (pretty(self.eff_nozzle), user_tail))
        else:
            user = ("Print core: swap complete. %s nozzle detected.%s"
                    % (pretty(self.eff_nozzle), user_tail))
        self._say(gcmd, user,
                  "printcore: swap done: %s%s, hotend %.1f C. Normal "
                  "protection restored.%s"
                  % (self.eff_nozzle,
                     " (ID bypassed)" if self._bypass_active() else "",
                     avg, dev_tail))

    def _arm_swap_timer(self, secs):
        now = self.reactor.monotonic()
        self._swap_deadline = now + secs
        self._swap_warned = secs <= 2. * SWAP_WARN_SECS
        first = self._swap_deadline if self._swap_warned \
            else self._swap_deadline - SWAP_WARN_SECS
        self.reactor.update_timer(self.swap_timer, first)

    def _swap_timeout_cb(self, eventtime):
        if not self.swap_active:
            return self.reactor.NEVER
        if not self._swap_warned and eventtime < self._swap_deadline:
            self._swap_warned = True
            self._say(None,
                      "!! Print core: %.0f s left to finish the swap. Add "
                      "time with PRINTCORE_SWAP_EXTEND (or the 'more time' "
                      "button)." % SWAP_WARN_SECS,
                      "!! printcore: %.0f s left in swap mode; "
                      "PRINTCORE_SWAP_EXTEND adds time" % SWAP_WARN_SECS)
            return self._swap_deadline
        self.fault(
            "Print core [PC-07]: the swap time limit was reached without "
            "finishing the swap. Printer stopped because the temperature "
            "protection is off during a swap, so it cannot wait any longer. "
            "Fix the print core, restart (FIRMWARE_RESTART) and try again.",
            "printcore: swap time limit reached; protection was off, "
            "stopping")
        return self.reactor.NEVER

    def cmd_SWAP_EXTEND(self, gcmd):
        if not self.swap_active:
            raise gcmd.error(self.M("Print core: no swap in progress.",
                                    "printcore: no swap in progress"))
        if not self.swap_timeout:
            raise gcmd.error(self.M(
                "Print core: this swap has no time limit.",
                "printcore: swap_timeout is 0, nothing to extend"))
        secs = gcmd.get_float('SECONDS', self.swap_timeout, minval=30.,
                              maxval=3600.)
        self._arm_swap_timer(secs)
        self._say(gcmd,
                  "Print core: swap time extended. %.0f s left." % secs,
                  "printcore: swap deadline reset to %.0f s from now"
                  % secs)

    # ------------------------------------------------------------------
    def cmd_STATUS(self, gcmd):
        if self.swap_active:
            user = "Print core: swap in progress."
        elif self.eff_state == PRESENT:
            user = "Print core: %s nozzle%s. Ready." % (
                pretty(self.eff_nozzle),
                " (ID check bypassed)" if self._bypass_active() else "")
        elif self.state == UNKNOWN:
            user = "Print core: not recognized. Heating disabled."
        else:
            user = "Print core: none detected. Heating disabled."
        if self.id_fault_suspected and not self._bypass_active() \
                and not self.swap_active:
            user += " ID signal fault suspected: see PRINTCORE_ID_BYPASS."
        self._say(
            gcmd, user,
            "printcore: id_state=%s eff_state=%s nozzle=%s adc=%s "
            "id_bypass=%s id_fault_suspected=%s swap_active=%s "
            "heating_allowed=%s motion_locked=%s log_level=%d"
            % (self.state, self.eff_state, self.eff_nozzle or "-",
               "-" if self.counts is None else "%.0f" % self.counts,
               self.bypass_nozzle or "-", self.id_fault_suspected,
               self.swap_active, self.heating_allowed(), self.swap_active,
               self.log_level))

    def cmd_REQUIRE(self, gcmd):
        want = gcmd.get('NOZZLE', None)
        if self.swap_active:
            raise gcmd.error(self.M("Print core: a swap is in progress.",
                                    "printcore: swap in progress"))
        if self.eff_state != PRESENT:
            raise gcmd.error(self.M(
                "Print core: no print core detected.",
                "printcore: no valid print core (state=%s)" % (self.state,)))
        if want is not None and want != self.eff_nozzle:
            raise gcmd.error(self.M(
                "Print core: a different print core is mounted than "
                "expected.",
                "printcore: mounted '%s', expected '%s'"
                % (self.eff_nozzle, want)))

    def _macro_vars(self, name):
        macro = self.printer.lookup_object('gcode_macro ' + name, None)
        return getattr(macro, 'variables', None) or {}

    def get_status(self, eventtime):
        # Tudo o que o LCD precisa num só objeto ("printcore"). Valores
        # simples (str/float/bool/list) para serem serializáveis em JSON.
        phase = self._macro_vars('PRINT_CORE_CHANGE').get('phase', 'idle')
        pos = self._macro_vars('PRINT_CORE_POSITIONS')
        try:
            swap_pos = {'x': float(pos['swap_x']), 'y': float(pos['swap_y']),
                        'z': float(pos['swap_z'])}
        except (KeyError, TypeError, ValueError):
            swap_pos = {}
        left = 0.
        if self.swap_active and self.swap_timeout:
            left = max(0., self._swap_deadline - eventtime)
        return {
            # print core
            'state': self.eff_state,            # present|absent|unknown
            'present': self.eff_state == PRESENT,
            'nozzle': self.eff_nozzle,          # nome da tabela
            'nozzle_label': pretty(self.eff_nozzle) if self.eff_nozzle else "",
            'nozzles': self._nozzle_names(),    # nomes válidos (por ADC)
            # linha de ID
            'id_state': self.state,
            'id_counts': self.counts if self.counts is not None else 0.,
            'id_bypass': self._bypass_active(),
            'bypass_nozzle': self.bypass_nozzle,
            'id_fault_suspected': self.id_fault_suspected,
            # troca
            'phase': str(phase),                # idle|unloading|preparing
            'swap_active': self.swap_active,
            'swap_timeout_left': left,          # s; 0 = sem limite/inativo
            'swap_position': swap_pos,
            'motion_locked': self.swap_active,
            'heating_allowed': self.heating_allowed(),
            'log_level': self.log_level,
            'heater': self.heater_name,
        }


def load_config(config):
    return PrintCore(config)
