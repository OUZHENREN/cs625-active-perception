# Static checks

`contract_checks.py` is host-side and does not require ROS. It verifies the Phase 0–1 package set, required governance files, package XML names, safe launch defaults and Python syntax.

Run from the repository root:

```bash
python3 test/contract_checks.py
```

Ubuntu VM build, xacro, launch, TF, controller and sensor checks remain separate gates.

