"""Simulation of the print-hold logic with fake Klipper objects."""
import configparser, importlib.util, os, sys, types, unittest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

extras = types.ModuleType("extras")
extras.__path__ = []
spi = types.ModuleType("extras.spi_temperature")
class MAX31865: pass
spi.MAX31865 = MAX31865
spi.REPORT_TIME = 0.3
spi.MAX_INVALID_COUNT = 3
extras.spi_temperature = spi
sys.modules["extras"] = extras
sys.modules["extras.spi_temperature"] = spi
spec = importlib.util.spec_from_file_location("extras.printcore", os.path.join(REPO, "printcore.py"))
pc = importlib.util.module_from_spec(spec)
sys.modules["extras.printcore"] = pc
spec.loader.exec_module(pc)


class CmdErr(Exception): pass

class Reactor:
    NEVER = 9e99
    t = 100.0
    def monotonic(self): return self.t
    def register_timer(self, cb, when): return object()
    def update_timer(self, t, when): pass
    def __init__(self): self.queued = []
    def register_async_callback(self, cb): self.queued.append(cb)
    def run_queued(self):
        q, self.queued = self.queued, []
        for cb in q: cb(self.t)
    def pause(self, w): self.t = w

class Gcode:
    def __init__(self): self.out = []; self.cmds = {}
    def register_command(self, n, f, desc=None):
        o = self.cmds.get(n); self.cmds[n] = f; return o
    def respond_info(self, m): self.out.append(m)
    def respond_raw(self, m): self.out.append(m)
    def run_script(self, s): pass
    def run_script_from_command(self, s): pass

class Stats:
    state = "standby"
    def get_status(self, e): return {"state": self.state}

class ADC:
    def setup_adc_sample(self, report_time, a, b, minval=0, maxval=1): pass
    def setup_adc_callback(self, cb): self.cb = cb

class Pins:
    def setup_pin(self, t, p): return ADC()

class Heater:
    target_temp = 0.0; last_temp = 25.0; smoothed_temp = 25.0; last_temp_time = 100.0
    last_pwm_value = 0.0; max_temp = 300.0
    def __init__(self, sensor): self.sensor = sensor
    def set_temp(self, d): self.target_temp = d
    def alter_target(self, d): self.target_temp = d

class MCU:
    def estimated_print_time(self, t): return t
    def lookup_command(self, f): return None

class Printer:
    def __init__(self):
        self.reactor = Reactor(); self.gcode = Gcode(); self.stats = Stats()
        self.shutdowns = []
        self.objs = {"gcode": self.gcode, "pins": Pins(), "print_stats": self.stats}
    def get_reactor(self): return self.reactor
    def lookup_object(self, n, d=None): return self.objs.get(n, d)
    def register_event_handler(self, e, f): pass
    def invoke_shutdown(self, m): self.shutdowns.append(m)
    command_error = CmdErr
    config_error = CmdErr

class Config:
    def __init__(self, p, vals): self.p, self.v = p, vals
    def get_printer(self): return self.p
    def get(self, k, d=None): return self.v.get(k, d)
    def getint(self, k, d=None, **kw): return int(self.v.get(k, d))
    def getfloat(self, k, d=None, **kw): return float(self.v.get(k, d))
    def getboolean(self, k, d=None): return d
    def error(self, m): return Exception(m)


cp = configparser.RawConfigParser(strict=False, inline_comment_prefixes=None, comment_prefixes=("#",))
with open(os.path.join(REPO, "printcore.cfg"), encoding="utf-8") as f:
    cp.read_file(f)
VALS = dict(cp["printcore"])
C04 = 2815 / 4096.0
C06 = 2232 / 4096.0
OPEN = 4095 / 4096.0
ZERO = 0.0
NOISE = 3300 / 4096.0


class T(unittest.TestCase):
    def setUp(self):
        self.pr = Printer()
        self.m = pc.PrintCore(Config(self.pr, VALS))
        s = MAX31865(); s.mcu = MCU()
        self.h = Heater(s)
        self.pr.objs["heaters"] = types.SimpleNamespace(lookup_heater=lambda n: self.h)
        self.m._handle_connect()
        self.m._started = True
        self.feed(C04)
        self.pr.gcode.out.clear()

    def feed(self, v, n=3):
        for _ in range(n):
            self.pr.reactor.t += 0.3
            self.h.last_temp_time = self.pr.reactor.t
            self.m._id_sample(v)

    def tick(self, n=1):
        for _ in range(n):
            self.pr.reactor.t += 1.0
            self.h.last_temp_time = self.pr.reactor.t
            self.m._monitor_cb(self.pr.reactor.t)

    def start(self, heat=True):
        self.pr.stats.state = "printing"
        if heat:
            self.h.set_temp(220.0)
        self.tick()

    def end(self, state="complete"):
        self.pr.stats.state = state
        self.tick()

    def warns(self):
        return [m for m in self.pr.gcode.out if m.startswith("!!")]

    def test_clean_print_says_nothing(self):
        self.start(); self.feed(C04, 30); self.end()
        self.assertEqual(self.pr.gcode.out, [])
        self.assertEqual(self.m.print_nozzle, "")

    def test_id_lost_keeps_nozzle_and_is_silent(self):
        self.start()
        for v in (OPEN, C06, ZERO, NOISE):
            self.feed(v, 6)
            self.tick(7)
            st = self.m.get_status(self.pr.reactor.t)
            self.assertEqual(st["nozzle"], "0.4_standard")
            self.assertTrue(st["present"])
            self.assertTrue(st["heating_allowed"])
        self.assertEqual(self.pr.gcode.out, [])
        self.assertEqual(self.pr.shutdowns, [])

    def test_stale_rtd_with_id_change_does_not_shut_down_in_print(self):
        self.start()
        self.h.last_temp_time = 0.0
        for _ in range(3):
            self.m._id_sample(OPEN)
        self.assertEqual(self.pr.shutdowns, [])

    def test_pause_heater_off_then_resume_with_dead_id(self):
        self.start()
        self.pr.stats.state = "paused"
        self.h.set_temp(0.0)
        self.feed(OPEN, 6)
        self.h.set_temp(220.0)
        self.assertEqual(self.h.target_temp, 220.0)

    def test_report_when_flapping_but_ok_at_end(self):
        self.start()
        self.feed(OPEN, 3); self.feed(C04, 3)
        self.feed(C06, 3); self.feed(C04, 3)
        self.end()
        w = self.warns()
        self.assertEqual(len(w), 1, w)
        self.assertIn("unstable", w[0]); self.assertIn("2 times", w[0])
        self.assertEqual(self.m.id_print_faults, 2)

    def test_single_spike_is_not_reported(self):
        self.start()
        self.feed(OPEN, 2); self.feed(C04, 3); self.end()
        self.assertEqual(self.warns(), [])

    def test_varying_noise_never_debounces_but_is_counted(self):
        self.start()
        for v in (OPEN, ZERO, NOISE):
            self.feed(v, 1)
        self.feed(C04, 3); self.end()
        self.assertEqual(self.m.id_print_faults, 1)

    def test_report_reads_zero_at_end_and_heating_blocked_after(self):
        self.start(); self.feed(ZERO, 3); self.end()
        w = self.warns()
        self.assertEqual(len(w), 1, w); self.assertIn("reads 0", w[0])
        self.assertEqual(self.m.eff_state, pc.UNKNOWN)
        self.h.target_temp = 0.0
        with self.assertRaises(CmdErr):
            self.h.set_temp(200.0)

    def test_report_open_and_other_nozzle(self):
        self.start(); self.feed(OPEN, 3); self.end("cancelled")
        self.assertIn("is not being read", self.warns()[0])
        self.pr.gcode.out.clear(); self.feed(C04, 3); self.pr.gcode.out.clear()
        self.start(); self.feed(C06, 3); self.end("error")
        self.assertIn("now reads 0.6 mm standard", self.warns()[0])

    def test_flap_just_before_first_tick_holds_good_nozzle(self):
        self.pr.stats.state = "printing"
        self.feed(OPEN, 3)
        self.assertEqual(self.m.print_nozzle, "0.4_standard")

    def test_no_valid_id_at_start_cannot_heat_even_if_preheated(self):
        self.h.set_temp(150.0)
        self.feed(OPEN, 3)
        self.pr.stats.state = "printing"
        self.tick()
        self.assertEqual(self.m.print_nozzle, "")
        with self.assertRaises(CmdErr):
            self.h.set_temp(220.0)

    def test_id_recovers_after_start_gets_held(self):
        self.feed(OPEN, 3)
        self.pr.stats.state = "printing"
        self.tick(); self.feed(C04, 3); self.tick()
        self.assertEqual(self.m.print_nozzle, "0.4_standard")

    def test_bypass_print_not_reported(self):
        self.feed(OPEN, 3)
        self.m.bypass_nozzle = "0.4_hightemp"
        self.start(); self.feed(OPEN, 6); self.end()
        self.assertEqual(self.warns(), [])
        self.assertEqual(self.m.bypass_nozzle, "0.4_hightemp")

    def test_outside_print_still_shuts_down_on_stale_rtd(self):
        self.h.target_temp = 200.0
        self.h.last_temp_time = 0.0
        for _ in range(3):
            self.m._id_sample(OPEN)
        self.assertEqual(len(self.pr.shutdowns), 1)

    def test_outside_print_flap_while_heating_keeps_target_change(self):
        self.h.set_temp(200.0)
        self.feed(OPEN, 3)
        self.h.set_temp(210.0)
        self.assertEqual(self.h.target_temp, 210.0)

    def test_cannot_swap_during_print(self):
        self.start()
        self.h.target_temp = 0.0
        g = types.SimpleNamespace(error=lambda m: CmdErr(m), respond_info=print)
        with self.assertRaises(CmdErr):
            self.m._check_idle(g)

    def test_adc_callback_defers_to_reactor(self):
        # Serial thread: nothing may run until the reactor picks it up
        adc = self.m.id_adc
        self.pr.reactor.queued.clear()
        adc.cb([(0.0, OPEN)] * 3)
        self.assertEqual(self.m.state, pc.PRESENT)
        self.assertEqual(len(self.pr.reactor.queued), 1)
        self.pr.reactor.run_queued()
        self.assertEqual(self.m.state, pc.ABSENT)

    def test_adc_callback_shutdown_runs_on_reactor(self):
        self.h.target_temp = 200.0
        self.h.last_temp_time = 0.0
        self.m.id_adc.cb([(0.0, OPEN)] * 3)
        self.assertEqual(self.pr.shutdowns, [])
        self.pr.reactor.run_queued()
        self.assertEqual(len(self.pr.shutdowns), 1)

    def test_adc_callback_old_api_and_bad_samples(self):
        self.pr.reactor.queued.clear()
        self.m.id_adc.cb([(0.0,), None])
        self.assertEqual(self.pr.reactor.queued, [])
        for _ in range(3):
            self.m._id_callback(0.0, OPEN)
        self.pr.reactor.run_queued()
        self.assertEqual(self.m.state, pc.ABSENT)

    def test_swap_probe_insert_and_remove(self):
        # Swap with no core: RTD silent, ID open
        self.feed(OPEN, 3)
        self.m.swap_active = True
        self.m._rtd = pc.RTD_GONE
        res = [("u", "d")]
        self.m._rtd_probe = lambda: res[0]
        self.pr.gcode.out.clear()
        self.m._probe_cb(0)
        self.assertEqual(self.m._rtd, pc.RTD_GONE)
        # RTD responds, ID still open: wait one probe, say nothing
        res[0] = None
        self.m._probe_cb(0)
        self.assertEqual(self.m._rtd, pc.RTD_NEW)
        self.assertEqual(self.pr.gcode.out, [])
        # Second probe: report the unreadable ID once
        self.m._probe_cb(0)
        self.assertEqual(self.m._rtd, pc.RTD_SEEN)
        self.assertEqual(len(self.pr.gcode.out), 1)
        self.assertIn("ID cannot be read", self.pr.gcode.out[0])
        self.m._probe_cb(0)
        self.assertEqual(len(self.pr.gcode.out), 1)
        # RTD stops: removed
        res[0] = ("u", "d")
        self.m._probe_cb(0)
        self.assertEqual(self.m._rtd, pc.RTD_GONE)
        self.assertIn("removed", self.pr.gcode.out[-1])

    def test_swap_probe_id_arrives_before_report(self):
        self.feed(OPEN, 3)
        self.m.swap_active = True
        self.m._rtd = pc.RTD_GONE
        self.m._rtd_probe = lambda: None
        self.m._probe_cb(0)
        self.assertEqual(self.m._rtd, pc.RTD_NEW)
        self.pr.gcode.out.clear()
        self.feed(C04, 3)
        self.assertEqual(self.m._rtd, pc.RTD_SEEN)
        self.assertFalse(any("cannot be read" in m for m in self.pr.gcode.out))

    def test_msg_fixes(self):
        src = open(os.path.join(REPO, "printcore.py"), encoding="utf-8").read()
        self.assertNotIn("(M104 S0) \"\n                    \"printcore:", src)
        self.assertNotIn('mounted"\n', src)
        cfg = open(os.path.join(REPO, "printcore.cfg"), encoding="utf-8").read()
        self.assertNotIn("PRINTCORE_LOG", cfg)
        self.assertNotIn("fica guardado", cfg)


unittest.main(verbosity=1)
