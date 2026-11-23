# An Embodiment-Adaptive RL Pipeline for Robotics Locomotion with a Morphology Gap

A single-file, CPU-only Webots controller that takes a humanoid from "nothing is known about the joints" to a walking gait, with no per-robot hand tuning. It is evaluated on two very different robots in simulation: the **DARwIn-OP** (0.339 m, small hobby-class servos) and the **Boston Dynamics Atlas** (0.921 m, large torques).

The controller is pure Python (standard library only). It needs no GPU, no NumPy, no PyTorch and no other packages.

> Paper: *An Embodiment-Adaptive RL Pipeline for Robotics Locomotion with a Morphology Gap* (Jude Onyenze, University of Texas at Dallas).

## What it does

Each stage is cached, so a restart skips the stages that are already finished.

1. **Calibrate.** Reads the joint limits, works out which way each leg joint bends, and finds a standing pose that stays upright. It tries the hip and ankle sign combinations and keeps the one that holds.
2. **Search for a gait.** A parameterised central-pattern-generator (CPG) gait is improved by random-search hill-climbing. Candidates are judged under small random command noise, so only gaits that survive a nudge are kept.
3. **Curriculum for hard robots.** For heavy robots such as Atlas, the search goes through marching in place, then growing stride lengths, then a balance-only retune.
4. **Humanize.** Adds human-like sway, arm swing and pelvis rotation, only while the robot stays up.
5. **Residual learning (optional).** A TD3 agent learns small corrections on top of the best gait. In our experiments this stage did **not** help; see the paper for the result and discussion.

## Repository layout

| Folder | Contents |
| --- | --- |
| `darwinRLwalker/` | Controller and Webots project for the DARwIn-OP |
| `atlas_simple_walk/` | Controller and Webots project for the Atlas |

The controller file is `darwin_rl.py`. The same pipeline runs for both robots; the robot is detected from its device names.

## Requirements

- **Webots R2025a** (the version used for all results)
- Python as configured in Webots (any recent Python 3; standard library only)
- A CPU. No GPU is used.

## Quick start

1. Install Webots R2025a.
2. Open the world file in the robot folder you want (`darwinRLwalker/` or `atlas_simple_walk/`).
3. Make sure the robot node has `supervisor TRUE` and `controller "darwin_rl"`. The file should live at `controllers/darwin_rl/darwin_rl.py` inside the project.
4. For speed, switch to **Fast** mode and turn rendering off.
5. Press Play. The controller prints what each stage decides.

Outputs are written next to the controller:

| File | Purpose |
| --- | --- |
| `darwin_state.json` | Calibration and best gaits (Atlas: `atlas_state.json`). Delete it to redo everything. |
| `darwin_rl_model.json` | Saved TD3 residual networks (Atlas: `atlas_rl_model.json`). |
| `training_log_v2.csv` | Training log (Atlas: `atlas_training_log.csv`). |

## Configuration

All settings are constants at the top of `darwin_rl.py`. The most useful ones:

| Setting | Default | Meaning |
| --- | --- | --- |
| `PROFILE` | `"auto"` | `"auto"`, `"darwin"`, `"atlas"`, or use `CUSTOM_PROFILE` for another humanoid |
| `SEED` | `1` | Random seed |
| `RUN_TRAINING` | `True` | `False` stops after the gait search and replays the gait |
| `OMNI` | `True` | Also search backward, left and right gaits |
| `HUMANIZE` | `True` | Run the humanize stage |
| `ROBUST_SEARCH` | `True` | Judge gaits under random command noise |
| `ROBUST_LEVELS` | `(0.003, 0.006, 0.010, 0.015)` | Noise levels (rad) used by the polish |
| `REDO` | `()` | Force stages to rerun, e.g. `("search",)` |
| `KEYBOARD_MODE` | `False` | Drive the robot with the keyboard instead of training |

### Important for reproducing from scratch

The Darwin-OP configuration currently contains a `FORCE_GAIT` dictionary that restores a previously found gait once. **Set `FORCE_GAIT = None` and delete `darwin_state.json`** if you want the search to start from scratch.

## Keyboard control

Set `KEYBOARD_MODE = True`, click the 3D view so it receives keys, then:

| Key | Action |
| --- | --- |
| `W` / Up | Forward |
| `S` / Down | Backward |
| `Q` | Left |
| `E` | Right |

Backward, left and right gaits exist only if `OMNI = True` and the search has finished for them.

## Reproducing the paper's results

| Result | How to run |
| --- | --- |
| Calibration (both robots) | Delete the state file and run; the sign-combination scores are printed |
| Atlas gait, curriculum and noise polish | Run the Atlas project with the default settings |
| TD3 residual (negative result) | Leave `RUN_TRAINING = True`; results are in the training log |

[TODO: add the exact figure/table-to-run mapping, the number of seeds used for each result, and expected wall-clock time.]

## Known limitations

- Results are from the Webots simulator only. Nothing here is tested on hardware.
- Two robots, with [N] seeds each [TODO: fill in].
- The TD3 residual did not improve on the tuned gait in our experiments.
- Wall-clock time for the Atlas search was not logged.

## Citation

```bibtex
@misc{onyenze2026embodiment,
  author = {Onyenze, Jude},
  title  = {An Embodiment-Adaptive RL Pipeline for Robotics Locomotion with a Morphology Gap},
  year   = {2026},
  note   = {Code: https://github.com/MrO9/An-Embodiment-Adaptive-RL-Pipeline-for-Robotics-Locomotion-with-a-Morphology-Gap}
}
```

## Contact

Jude Onyenze, Department of Mechanical Engineering, University of Texas at Dallas.
chisimdi.onyenze@utdallas.edu

