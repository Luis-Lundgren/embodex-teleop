# Contributing to Embodex Teleop

Thank you for your interest in contributing to Embodex Teleop! We welcome contributions of all kinds: bug fixes, documentation improvements, new challenge tasks, new hardware robot models, and performance enhancements.

---

## Code of Conduct

Please adhere to our [Code of Conduct](CODE_OF_CONDUCT.md) in all project interactions.

---

## Development Setup

1. Fork and clone the repository with submodules:
   ```bash
   git clone --recurse-submodules https://github.com/Luis-Lundgren/embodex-teleop.git
   cd embodex-teleop
   ```

2. Create and activate a Python virtual environment:
   ```bash
   python3 -m venv .venv
   source .venv/bin/activate
   ```

3. Install base dependencies and packages in editable mode:
   ```bash
   pip install -r requirements-base.txt
   pip install -e ./vendor/telegrip
   pip install -e .
   ```

4. Run the automated smoke test:
   ```bash
   pytest tests/
   ```

---

## Architectural Guidelines

- **TeleGrip Boundary**: TeleGrip in `vendor/telegrip` handles raw kinematics, PyBullet IK computation, and serial motor interfaces. Avoid putting Embodex database, UI, or platform-specific logic inside `vendor/telegrip`.
- **Platform Separation**: Put all session recording, REST endpoints, database sync, and challenge tasks inside the `embodex` package.
- **Dependency Decoupling**: Keep the base runtime free of large ML frameworks (PyTorch, LeRobot). Any heavy ML dependencies belong in `[project.optional-dependencies] ml` and `requirements-ml.txt`.
- **Hardware Agnostic**: Ensure code runs gracefully with `--no-robot` in simulation/digital-twin mode.

---

## Pull Request Guidelines

1. Create a feature branch off `main` with a descriptive name (e.g., `feat/add-ur5-arm`, `fix/recorder-timestamp`).
2. Verify that existing smoke tests pass:
   ```bash
   pytest tests/
   ```
3. Add tests for any new endpoints, tasks, or recording formats.
4. Keep commit messages clear and descriptive following Conventional Commits (e.g., `feat:`, `fix:`, `docs:`, `refactor:`).
5. Open a Pull Request against the `main` branch.
