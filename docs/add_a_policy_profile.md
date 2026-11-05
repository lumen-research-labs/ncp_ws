# Add a policy profile

How to put a newly trained checkpoint on the robot as its own dashboard button. A profile is three things: a contract file under `config/`, a weights file in the upstream checkout, and a row in `config/topics.yaml`. The dashboard draws one button per row. The `m9` profile (2026-11-04, commits `29c3cc5` and `a6350d5`) is the worked case.

## Before you start

- The snapshot directory on the training box holds `cfc_final.zip` and `vecnormalize.pkl` (a barn-gym RecurrentPPO checkpoint, for example `runs/m9_gz/archive/s152000`).
- Read the run's observation width and SB3 action space from the checkpoint, never from memory: `RecurrentPPO.load(...).policy.action_space` and the shape of `vecnormalize.pkl`'s `obs_rms.mean`. A lumen gauntlet `barn_lab` checkpoint (2026-11-05 on) acts in [-1, 1]: the contract's bounds are then the task's physical bounds, and the export takes them with `--bounds vlo,vhi,wlo,whi` (the `dynabarn` profile is the worked case).
- The workstation has the two clones side by side ([`workstation_setup.md`](workstation_setup.md)).

## 1. Export the weights

1. Copy the exporter to the box and run it against the snapshot:

   ```bash
   scp ~/lumen/ncp/weights/export_from_sb3.py root@<box>:/root/barn-gym/
   ssh root@<box> 'cd /root/barn-gym && nice -n 15 .venv/bin/python export_from_sb3.py runs/<run>/archive/<snapshot> /tmp/actor_<name>.pt'
   ```

   The output ends with `max action diff vs SB3: 0.0` and `saved`. A diff above 1e-5 is a failed export: the script asserts and writes nothing.
2. Copy the file to `~/lumen/ncp/weights/actor_<name>.pt`, record `sha256sum`, and commit it in the upstream checkout with the run and snapshot in the message.

## 2. Write the contract file

1. Copy the closest file: `cp config/interface.yaml config/interface_<name>.yaml` for a 720-beam checkpoint, `interface_m4.yaml` for a 32-beam one.
2. Set a new `interface_version` and add a change-log entry at the top.
3. Fill the `checkpoint` block: `file`, `sha256`, `run`, `snapshot`, `trained_on`, `heldout_success`, `exported`.
4. Set the `action.layout` ranges to the SB3 action space and describe the previous-action normalization that follows (`(a - bias) / scale`).
5. Leave the `observation` block alone unless the lidar layout changed. A different beam count changes `total_dim`; a different action range is only the step above.
6. Add the version to `_SUPPORTED_INTERFACE_VERSIONS` in `src/rm_policy/rm_policy/policy_node.py`.

`policy_runner.py` refuses bounds wider than the vendored actor's clip ([-1, 2], [-3.14, 3.14]) at startup. A checkpoint trained past them needs the vendored module updated first.

## 3. Register the profile

1. `config/topics.yaml`: a `command_sources` row named `<name>` with `topic: /policy/<name>/cmd_vel`, `kind: policy`, `node: policy_node_<name>`, `debug: /ncp_debug/<name>`, and a description (the button tooltip).
2. `src/rm_bringup/launch/ncp.launch.py`: `_PROFILES["<name>"] = ("interface_<name>.yaml", "actor_<name>.pt")`. Add the name to `_DEFAULT_PROFILES` if it should run by default; `m8v2` stays first.
3. `scripts/deploy_upstream.sh`: add `weights/actor_<name>.pt` to `CRITICAL`.
4. `scripts/record_run.sh`: add the two topics.

## 4. Extend the pins

Run `python3 -m pytest tests/` and follow the failures. They name every list the profile must appear in: `test_lidar_contract.py` (`PROFILES`), `test_source_per_policy_launch.py`, `test_command_sources.py`, `test_dashboard_sources.py`, and `test_action_contract.py` for the bounds. Add the row to [`interface_versions.md`](interface_versions.md), to the tables in [`operating_guide.md`](operating_guide.md) and to `src/rm_bringup/README.md`.

## 5. Deploy and check

1. `bash scripts/deploy_upstream.sh`, then `bash scripts/deploy_to_robot.sh`. The first prints `OK` with the checksum of each weights file.
2. On the robot: `colcon build --symlink-install --packages-select rm_policy rm_bringup`, then `scripts/stack_down.sh`, `scripts/prep_robot.sh`, `scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true`.
3. In `/tmp/ncp_stack.log` the new node's start line names the version, the beam count and the action bounds, and the next line its checkpoint sha256. The button appears beside the others on the dashboard and `/api/state` lists the source.
4. The floor test is the operator's, with KILL in hand: select the source, ARM, a goal 1.5 to 3 m ahead, then one to the side.
