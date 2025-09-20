# safeplan/main.py
import json
import os
import sys
import subprocess
import time
import gc  # NEW

from .core.logger import Logger

from .algos.a_star import AStar
from .algos.voronoi_planner import VoronoiPlanner
from .algos.upp import UPP
from .algos.rrt import RRT
from .algos.sdf_astar import SDFAStar
from .algos.cbf_rrt import CBFRRT
from .algos.optimized_astar import OptimizedAStar
from .algos.fs_planner import FSPlanner
from .algos.safe_astar import SafeAStar

from .envs.generate_grid import GenerateGrid

from .evals.path_cost import PathCost
from .evals.nodes_in_path import NodesInPath
from .evals.optimal_deviation import OptimalDeviation
from .evals.distance_to_goal import DistanceToGoal
from .evals.jerk_per_meter import JerkPerMeter
from .evals.turning_angle import TurningAngle
from .evals.minimum_clearance import MinimumClearance
from .evals.average_minimum_clearance import AverageMinimumClearance
from .evals.clearance_variability import ClearanceVariability
from .evals.danger_violations import DangerViolations
from .evals.optisafe_index import OptiSafeIndex
try:
    import psutil
    _PROC = psutil.Process()
except Exception:
    _PROC = None

def _log_rss(tag):
    if _PROC is None:
        return
    try:
        rss = _PROC.memory_info().rss / (1024 * 1024)
        print(f"[mem] {tag}: parent RSS={rss:.1f} MB")
    except Exception:
        pass


# --- NEW: helper to return memory to the OS (glibc) ---
import ctypes, ctypes.util  # NEW
_LIBC = None  # NEW
def _malloc_trim():  # NEW
    """Ask glibc to return free heap pages to the OS; harmless elsewhere."""
    global _LIBC
    try:
        if _LIBC is None:
            path = ctypes.util.find_library("c")
            if not path:
                return
            _LIBC = ctypes.CDLL(path)
        _LIBC.malloc_trim(0)
    except Exception:
        pass
# --- end NEW ---

class SafePlan:
    def __init__(self, runConfigPath):
        self.runConfigPath = runConfigPath

        with open(self.runConfigPath) as file:
            data = json.load(file)

        self.outputDir = "outputs"
        self.envsDetails = data["envDetails"]
        self.algosDetails = data["algoDetails"]
        self.evalsDetails = data["evalDetails"]
        self.runDetails = data["runDetails"]
        self.iteration = 0
        self.isSuccessEval = 0
        self.isPlanningTimeEval = 0
        self.isPeakMemoryEval = 0  # keep attribute but we won't use it

        try:
            self.syncEvery = int(os.environ.get("SAFEPLAN_SYNC_EVERY", "0"))
        except Exception:
            self.syncEvery = 0

        self.envs = []
        self.algos = []
        self.evals = []
        self.scenerios = []

        self.setUpAlgos()
        self.setUpEvals()
        self.setUpEnvs()

        self.logger = Logger(self.runDetails, self.algos, self.outputDir)
        self.runPath = os.path.join(self.outputDir, self.runDetails)

    # ---------- child helpers (no heavy work in parent) ----------

    def _get_pairs_count_via_child(self, env_idx: int) -> int:
        import re

        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(pkg_dir)

        payload = json.dumps({"run_config": self.runConfigPath, "env_idx": int(env_idx)}, ensure_ascii=False)

        # Child: load run_config.json directly; build env list the same way setUpEnvs does.
        child_code = (
            "import os, json\n"
            "from safeplan.envs.generate_grid import GenerateGrid\n"
            "args = json.loads(os.environ['SAFEPLAN_CHILD_ARGS'])\n"
            "with open(args['run_config']) as f:\n"
            "    data = json.load(f)\n"
            "envsDetails = data['envDetails']\n"
            "env_specs = []\n"
            "for k in envsDetails:\n"
            "    if 'generateGrid' in k:\n"
            "        for p in k['generateGrid']:\n"
            "            env_specs.append({'type':'generateGrid','name': p['name']})\n"
            "spec = env_specs[int(args['env_idx'])]\n"
            "env_obj = GenerateGrid(spec['name'])\n"
            "(grid, cellSize, envName, envDes, startGoalPairs) = env_obj.getmap()\n"
            "print('PAIR_COUNT=' + str(len(startGoalPairs)))\n"
        )

        env = os.environ.copy()
        env["SAFEPLAN_CHILD_ARGS"] = payload
        env["PYTHONPATH"] = project_root + (os.pathsep + env.get("PYTHONPATH", ""))
        # lean children
        env.setdefault("MALLOC_ARENA_MAX", "2")
        env.setdefault("MALLOC_TRIM_THRESHOLD_", "131072")
        env.setdefault("MALLOC_TOP_PAD_", "0")
        env.setdefault("MALLOC_MMAP_THRESHOLD_", "131072")
        env.setdefault("PYTHONMALLOC", "malloc")
        env.setdefault("PYTHONDONTWRITEBYTECODE", "1")
        env.setdefault("OMP_NUM_THREADS", "1")
        env.setdefault("OPENBLAS_NUM_THREADS", "1")
        env.setdefault("MKL_NUM_THREADS", "1")

        res = subprocess.run(
            [sys.executable, "-X", "utf8", "-c", child_code],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,        # capture for debugging
            stdin=subprocess.DEVNULL,
            text=True,
            encoding="utf-8",
            errors="replace",
            env=env,
            check=False,
            close_fds=True,
        )
        if res.returncode != 0:
            raise RuntimeError(
                f"pair-count child failed for env {env_idx}\n"
                f"stdout:\n{res.stdout}\n\nstderr:\n{res.stderr}"
            )

        count = None
        for line in res.stdout.splitlines():
            m = re.search(r"PAIR_COUNT=(\d+)\s*$", line)
            if m:
                count = int(m.group(1))
                break
        if count is None:
            raise ValueError(f"Could not parse pair count from child output:\n{res.stdout}")
        return count

    def _run_job_subprocess(self, env_idx: int, pair_idx: int, algo_name: str, iter_id: int):
        module_path = self.__class__.__module__
        pkg_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(pkg_dir)

        payload = json.dumps(
            {
                "run_config": self.runConfigPath,
                "env_idx": int(env_idx),
                "pair_idx": int(pair_idx),
                "algo_name": str(algo_name),
                "iter_id": int(iter_id),
            },
            ensure_ascii=False,
        )

        # Child: print [skip]/[ok]
        child_code = (
            "import os, json, time, gc, contextlib\n"
            "mod = __import__(os.environ['SP_MOD'], fromlist=['SafePlan','GenerateGrid'])\n"
            "SafePlan = getattr(mod, 'SafePlan')\n"
            "GenerateGrid = getattr(mod, 'GenerateGrid', None)\n"
            "args = json.loads(os.environ['SAFEPLAN_CHILD_ARGS'])\n"
            "sf = SafePlan(args['run_config'])\n"
            "sf.iteration = int(args['iter_id'])\n"
            "env_idx = int(args['env_idx']); pair_idx = int(args['pair_idx'])\n"
            "algo_name = args['algo_name']\n"
            "matches = [a for a in sf.algos if sf.getObjAndClass(a)[0] == algo_name]\n"
            "if not matches: raise SystemExit('Unknown algo_name: ' + str(algo_name))\n"
            "algoName, algoObj = sf.getObjAndClass(matches[0])\n"
            "if sf.iterationRanCheck(sf.iteration, algoName):\n"
            "    print('[skip] iter_%d %s already exists' % (sf.iteration, algoName)); raise SystemExit(0)\n"
            "with open(os.devnull, 'w') as devnull, contextlib.redirect_stdout(devnull), contextlib.redirect_stderr(devnull):\n"
            "    name, spec = sf.getObjAndClass(sf.envs[env_idx])\n"
            "    if isinstance(spec, dict) and spec.get('type') == 'generateGrid' and GenerateGrid is not None:\n"
            "        env_obj = GenerateGrid(spec['name'])\n"
            "    else:\n"
            "        env_obj = spec\n"
            "    (grid, cellSize, envName, envDes, startGoalPairs) = env_obj.getmap()\n"
            "    pair = startGoalPairs[pair_idx]\n"
            "    t0 = time.perf_counter(); pathData = algoObj.plan(pair['start'], pair['goal'], grid); t1 = time.perf_counter()\n"
            "    diff_ms = (t1 - t0) * 1000.0\n"
            "    evalData = {}\n"
            "    for ev in sf.evals:\n"
            "        nameEval, objEval = sf.getObjAndClass(ev)\n"
            "        evalData[nameEval] = objEval.eval(pair['start'], pair['goal'], grid, cellSize, pathData[1])\n"
            "    if getattr(sf, 'isSuccessEval', 0): evalData['SuccessRate'] = pathData[0]\n"
            "    if getattr(sf, 'isPlanningTimeEval', 0): evalData['PlanningTime'] = diff_ms\n"
            "    sf.logger.log(sf.iteration, algoName, pathData, evalData, pair,\n"
            "                  (grid, cellSize, envName, envDes, startGoalPairs))\n"
            "    del pathData, evalData; gc.collect()\n"
            "print('[ok] iter_%d %s done' % (sf.iteration, algoName))\n"
        )

        env = os.environ.copy()
        env["SP_MOD"] = module_path
        env["SAFEPLAN_CHILD_ARGS"] = payload
        env["PYTHONPATH"] = project_root + (os.pathsep + env.get("PYTHONPATH", ""))
        # --- NEW: allocator & runtime knobs for leaner children ---
        env.setdefault("MALLOC_ARENA_MAX", "2")
        env.setdefault("MALLOC_TRIM_THRESHOLD_", "131072")
        env.setdefault("MALLOC_TOP_PAD_", "0")
        env.setdefault("MALLOC_MMAP_THRESHOLD_", "131072")
        env.setdefault("PYTHONMALLOC", "malloc")
        env.setdefault("PYTHONDONTWRITEBYTECODE", "1")
        env.setdefault("OMP_NUM_THREADS", "1")
        env.setdefault("OPENBLAS_NUM_THREADS", "1")
        env.setdefault("MKL_NUM_THREADS", "1")
        # --- end NEW ---

        subprocess.run(
            [sys.executable, "-X", "utf8", "-c", child_code],  # NEW: -S for lean child
            env=env,
            check=False,
            close_fds=True,
            stdin=subprocess.DEVNULL,  # NEW
            stdout=subprocess.DEVNULL,  # NEW: no pipe allocation
            stderr=subprocess.DEVNULL,  # NEW
        )

        _malloc_trim()  # NEW: return freed pages after each child
        gc.collect()    # NEW: encourage prompt object finalization

    # ---------- setup ----------

    def setUpAlgos(self):
        for k in self.algosDetails:
            if k["name"] == "AStar":
                self.algos.append(("AStar", AStar()))
            if k["name"] == "OptimizedAStar":
                self.algos.append(
                    (
                        "OptimizedAStar",
                        OptimizedAStar(
                            k["args"]["turnPenaltyCoefficients"],
                            k["args"]["safetyDistGridRadius"],
                            k["args"]["maxInflateIter"],
                            k["args"]["pointSamples"],
                        ),
                    )
                )
            if k["name"] == "SDFAStar":
                self.algos.append(("SDFAStar", SDFAStar(k["args"]["k1"], k["args"]["k2"])))
            if k["name"] == "FSPlanner":
                self.algos.append(("FSPlanner",FSPlanner(k["args"]["pointSamples"],k["args"]["cw"],k["args"]["epsilon"],k["args"]["maxNeigh"])))
            if k["name"] == "SafeAStar":
                self.algos.append(("SafeAStar", SafeAStar(k["args"]["safetyFactor"],k["args"]["epsilon"])))
            if k["name"] == "RRT":
                self.algos.append(
                    (
                        "RRT",
                        RRT(k["args"]["maxIter"], k["args"]["goalSampleRate"], k["args"]["stepSize"], k["args"]["pointSamples"]),
                    )
                )
            if k["name"] == "CBFRRT":
                self.algos.append(
                    (
                        "CBFRRT",
                        CBFRRT(
                            k["args"]["maxIter"],
                            k["args"]["goalSampleRate"],
                            k["args"]["stepSize"],
                            k["args"]["pointSamples"],
                            k["args"]["gamma1"],
                            k["args"]["gamma2"],
                        ),
                    )
                )
            if k["name"] == "VoronoiPlanner":
                self.algos.append(("VoronoiPlanner", VoronoiPlanner(k["args"]["pointSamples"], k["args"]["knn"])))
            if k["name"] == "UPP":
                self.algos.append(("UPP", UPP(k["args"]["alpha"], k["args"]["beta"], k["args"]["radius"], k["args"]["epsilon"])))

    def setUpEvals(self):
        for k in self.evalsDetails:
            if k["name"] == "SuccessRate":
                self.isSuccessEval = 1
            if k["name"] == "PlanningTime":
                self.isPlanningTimeEval = 1
            if k["name"] == "PathCost":
                self.evals.append(("PathCost", PathCost()))
            if k["name"] == "NodesInPath":
                self.evals.append(("NodesInPath", NodesInPath(k["args"]["type"], k["args"]["epsilon"])))
            if k["name"] == "OptimalDeviation":
                self.evals.append(("OptimalDeviation", OptimalDeviation()))
            if k["name"] == "DistanceToGoal":
                self.evals.append(("DistanceToGoal", DistanceToGoal()))
            if k["name"] == "JerkPerMeter":
                self.evals.append(("JerkPerMeter", JerkPerMeter()))
            if k["name"] == "TurningAngle":
                self.evals.append(("TurningAngle", TurningAngle()))
            if k["name"] == "OptiSafeIndex":
                self.evals.append(("OptiSafeIndex", OptiSafeIndex(k["args"]["pointSamples"], k["args"]["knn"])))
            if k["name"] == "MinimumClearance":
                self.evals.append(("MinimumClearance", MinimumClearance(k["args"]["pointSamples"])))
            if k["name"] == "AverageMinimumClearance":
                self.evals.append(("AverageMinimumClearance", AverageMinimumClearance(k["args"]["pointSamples"])))
            if k["name"] == "ClearanceVariability":
                self.evals.append(("ClearanceVariability", ClearanceVariability(k["args"]["pointSamples"])))
            if k["name"] == "DangerViolations":
                self.evals.append(("DangerViolations", DangerViolations(k["args"]["pointSamples"], k["args"]["dangerRadius"])))

    def setUpEnvs(self):
        for k in self.envsDetails:
            if "generateGrid" in k:
                envList = k["generateGrid"]
                for p in envList:
                    name = "generateGrid_" + p["name"]
                    self.envs.append((name, {"type": "generateGrid", "name": p["name"]}))

    # ---------- utils ----------

    def getObjAndClass(self, data):
        name, obj = None, None
        for i in data:
            if type(i) == str:
                name = i
            else:
                obj = i
        return name, obj

    def iterationRanCheck(self, it, algoName):
        iterf = "iter_" + str(it) + ".json"
        path = os.path.join(self.logger.runPath, algoName, iterf)
        return os.path.exists(path)

    # ---------- benchmark ----------

    def benchmark(self):
        num_envs = len(self.envs)
        pair_counts = [self._get_pairs_count_via_child(ei) for ei in range(num_envs)]

        prefix = [0]
        for c in pair_counts:
            prefix.append(prefix[-1] + c)

        jobs_run = 0
        for env_idx in range(num_envs):
            for pair_idx in range(pair_counts[env_idx]):
                iter_id = 1 + prefix[env_idx] + pair_idx
                for algo in self.algos:
                    algoName, _ = self.getObjAndClass(algo)
                    if not self.iterationRanCheck(iter_id, algoName):
                        self._run_job_subprocess(env_idx, pair_idx, algoName, iter_id)
                        jobs_run += 1
                        if self.syncEvery and (jobs_run % self.syncEvery) == 0:
                            try:
                                os.sync()
                            except AttributeError:
                                pass
                            _malloc_trim()  # NEW
                            gc.collect()    # NEW

        print("Run Completed")
