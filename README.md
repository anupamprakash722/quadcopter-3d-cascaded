# quadcopter-3d-cascaded

> A full 6-DOF quadcopter — quaternion orientation, rigid-body dynamics, motor mixing
> and a cascaded controller — built from NumPy and Matplotlib, one small notebook at a
> time.
>
> **Project 5** of my [drone robotics learning roadmap](https://github.com/anupamprakash722/drone-robotics-learning-roadmap).

---

## What this is

Ten Jupyter notebooks in the same style as
[`math-foundations-sandbox`](https://github.com/anupamprakash722/math-foundations-sandbox):
one idea per section, small readable code cells, two exercises with worked solutions, a
mini-project with an animation, and a closing robotics connection.

Project 3 flew a quadcopter squashed into a vertical plane. Here the drone gets all
three dimensions, and with them the problems that only appear in 3-D: rotations that do
not commute, an angle representation with a hole in it, four motors instead of two, and
a controller that has to be built in layers.

| # | Notebook | What you build |
|---|---|---|
| 01 | `01_Two_Frames_in_3D.ipynb` | world and body frames, ENU/FLU, the X-configuration |
| 02 | `02_Rotation_Matrices_in_3D.ipynb` | $R_x, R_y, R_z$, composition, and non-commutativity |
| 03 | `03_Euler_Angles_and_Gimbal_Lock.ipynb` | roll/pitch/yaw, and the singularity demonstrated |
| 04 | `04_Meeting_the_Quaternion.ipynb` | axis-angle, Hamilton product, the double cover |
| 05 | `05_Integrating_Orientation.ipynb` | $\dot q = \tfrac12 q \otimes \omega$, and why to renormalise |
| 06 | `06_Rigid_Body_Dynamics_in_3D.ipynb` | the 13-state model, Euler's equation, RK4 |
| 07 | `07_Motors_and_Mixing.ipynb` | the mixing matrix from geometry, and saturation |
| 08 | `08_The_Inner_Loops.ipynb` | quaternion attitude error, rate control, recovering a tumble |
| 09 | `09_The_Full_Cascade.ipynb` | position → velocity → thrust+attitude → rate → motors |
| 10 | `10_Flying_a_Trajectory.ipynb` | minimum-jerk references and what feedforward is worth |

## Conventions, identical in all ten notebooks

* **ENU** world — $x$ East, $y$ North, $z$ Up. Gravity $[0,0,-9.81]$, hover $T = mg$.
* **FLU** body — $x$ nose, $y$ left, $z$ thrust. One consequence flagged in Notebook 01:
  a **positive pitch tips the nose down** and drives the drone forward. NED texts get
  the opposite sign; ours is verified numerically rather than argued about.
* 13-element state $[p(3), v(3), q(4), \omega(3)]$; quaternion $[q_w,q_x,q_y,q_z]$,
  Hamilton product, rotating **body → world**; $\omega$ in the body frame. Euler angles
  are used for display only, never as simulator state.
* X-configuration motors: **M1** front-right (CW), **M2** front-left (CCW), **M3**
  rear-left (CW), **M4** rear-right (CCW).
* Vehicle: $m = 1.0$ kg, arm 0.25 m, $I = \mathrm{diag}(0.01, 0.01, 0.02)$, motors capped
  at 6 N each — thrust-to-weight 2.45.

## Physical checks the notebooks run

| Check | Result |
|---|---|
| hover at $T = mg$ | drift exactly 0 |
| $T = 1.2\,mg$ for one second | +0.981 m |
| roll / pitch / yaw commands | rotate about their own axis only |
| quaternion vs rotation matrix | agree to 2.2e-16 |
| mixer round trip | 4.4e-16 |

## A few results worth the price of admission

* **Gimbal lock, demonstrated rather than asserted** (Notebook 03): two different Euler
  triples produce the identical rotation matrix at 90° of pitch, and a gentle 1 rad/s
  rotation demands Euler rates in the hundreds nearby.
* **The diagonal motor pair is not what you predict** (Notebook 07): M1 up and M3 down
  gives *zero* yaw — their drag torques cancel — while the same pair moved *together*
  gives yaw and no tilt. Same two motors, opposite result.
* **Yaw authority is ~11× weaker than roll** (Notebook 07), and the measured ratio
  matches the geometric prediction $a/d$ exactly.
* **Feedforward is worth about 20×** (Notebook 10): dropping $v_d$ and $a_d$ takes RMS
  tracking error from about 0.014 m to about 0.33 m on an identical path.
* **The tilt limit caps the command, not the response** (Notebook 09): a 35° limit is
  flown at roughly 40°, because the attitude loop has its own momentum.

## Running it

```bash
pip install -r requirements.txt
jupyter lab
```

Run each notebook top to bottom (Shift+Enter). Every notebook is self-contained, and all
ten contain 3-D animations rendered as an in-browser JavaScript player, so no ffmpeg is
needed.

## Tests

```bash
pytest -q
```

`tests/test_notebooks_run.py` executes every notebook in a clean kernel and fails on the
first error.

## This is an educational simulator

It assumes a rigid symmetric body, instant motors with no spin-up lag, no aerodynamic
drag beyond a torque constant, perfect noiseless state feedback, and no ground — the
drone in Notebook 06 falls to −16 m without complaint.
[`drone-ekf-estimation`](https://github.com/anupamprakash722/drone-ekf-estimation) is
about how far a real state estimate is from the truth, and
[`trajectory-optimization`](https://github.com/anupamprakash722/trajectory-optimization)
replaces Notebook 10's trajectory generator with an optimised one.

## License

MIT — see [LICENSE](LICENSE).
