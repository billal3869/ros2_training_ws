# Copilot / AI Agent Instructions for my_robot_controller

This repository is a small ROS 2 Python package using `ament_python` packaging. The goal of this file is to provide concise, actionable guidance so an AI coding agent can be productive immediately.

- **Project layout**: core code lives under `my_robot_controller/`; package manifest is `package.xml`; build/install metadata is in `setup.py`. Tests live in `test/` and use ament-style linters (`ament_flake8`, `ament_pep257`, etc.).

- **Big picture**: this is a ROS 2 Python package (see `package.xml` with `<build_type>ament_python</build_type>`). Nodes should be implemented with `rclpy`. The repository currently contains a placeholder script at `my_robot_controller/my_first_node.py` (this file is currently empty and needs a real `rclpy` node implementation).

- **How to build and test locally**:
  - Build workspace (from workspace root):

    ```bash
    colcon build --packages-select my_robot_controller
    ```

  - Run tests (ament/colcon test harness):

    ```bash
    colcon test --packages-select my_robot_controller
    colcon test-result --verbose
    ```

  - You can also run the package linters locally with pytest extras:

    ```bash
    python3 -m pytest -q
    ```

- **Entry-point convention**: this package uses `setup.py` with `console_scripts`. To make a node runnable with `ros2 run`, add an entry point in `setup.py`, for example:

  ```py
  entry_points={
      'console_scripts': [
          'my_first_node = my_robot_controller.my_first_node:main',
      ],
  }
  ```

- **Tests & linters**: the tests in `test/` are ament wrappers that enforce project-level policies (flake8, pep257, copyright). Keep test files as the authoritative source for CI expectations.

- **Patterns & conventions observed**:
  - Package name matches Python package directory: `my_robot_controller`.
  - `data_files` in `setup.py` must include `resource/` + `package.xml` for ROS 2 discovery — preserve that when changing packaging.
  - Dependencies declared in `package.xml` (e.g., `rclpy`) are the runtime dependencies; update both `package.xml` and `setup.py` when adding new deps.

- **What agents should not change without human review**:
  - `package.xml` and `setup.py` packaging blocks (data_files/resource entries). These are sensitive for ROS 2 discovery.
  - Test files under `test/` — updates may change CI behavior; discuss with the maintainer before modifying.

- **Quick actionable tasks examples** (use these as starting prompts):
  - "Implement a minimal rclpy node in `my_robot_controller/my_first_node.py` with a `main()` entrypoint and add a `console_scripts` entry in `setup.py`."
  - "Add a short README explaining how to run `ros2 run my_robot_controller my_first_node` after adding the `console_scripts` entry." 
  - "Run `colcon build` and `colcon test` locally and fix any immediate lint failures found by the ament tests."

- **Where to look for context**:
  - Package metadata: `package.xml`
  - Packaging and entry points: `setup.py`
  - Node implementation: `my_robot_controller/my_first_node.py`
  - CI/linter expectations: `test/*.py`

If any section is unclear or you want me to expand examples (e.g., a ready-to-run `my_first_node.py` implementation and the exact `setup.py` patch), tell me which part to flesh out next.
