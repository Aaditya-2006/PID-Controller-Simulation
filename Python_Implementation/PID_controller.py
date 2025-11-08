# importing necessary modules
import control as ct
import numpy as np
import matplotlib.pyplot as plt

''' SIMULATING PID CONTROLLER FOR SYSTEM Gs for time = t. '''

# This function returns time (for simulation), response (of system), other info (like risetime, overshoot)
def simulate_pid_with_metrics(Kp, Ki, Kd, Gs, t):
    # special case (if all gains are 0), simulating open loop response
    if Kp == 0 and Ki == 0 and Kd == 0:
        system_to_sim = Gs
    else:
        # C(s) = Kp + Ki/s + Kd*s = (Kd*s^2 + Kp*s + Ki) / s
        # now handling other special cases
        if Ki == 0 and Kd == 0: # P only
            Cs = ct.tf([Kp], [1])
        elif Ki == 0 and Kd != 0: # PD only 
            Cs = ct.tf([Kd, Kp], [1])
        elif Kd == 0: # PI only
            Cs = ct.tf([Kp, Ki], [1, 0])
        else: # full PID
            Cs = ct.tf([Kd, Kp, Ki], [1, 0])
        
        # creating closed loop system
        # G_cl(s) = C(s) * G(s) / (1 + C(s) * G(s))
        system_to_sim = ct.feedback(Cs * Gs, 1)
    
    # simulating step response
    time, response = ct.step_response(system_to_sim, T=t)

    # calculate performance metrics
    info = {}
    try:
        # using 2% settiling time for threshold
        info = ct.step_info(system_to_sim, T=t, SettlingTimeThreshold=0.02)

        # manually calculating steady state error for more accuracy
        final_value = response[-1]
        steady_state_error = 1.0 - final_value
        info['SteadyStateError'] = steady_state_error

    except Exception as e:
        print(f"Warning: Coud not calculate step_info for (Kp={Kp}, Ki={Ki}, Kd={Kd}). Error: {e}")
        # Manually calculate error if step_info fails
        final_value = response[-1]
        steady_state_error = 1.0 - final_value
        info = {'SteadyStateError': steady_state_error, 'Overshoot': 0.0, 'SettlingTime': 0.0, 'RiseTime': 0.0}
    
    return time, response, info


'''  EXECUTION  '''

if __name__ == "__main__":
    print("Starting PID Controller Simulation...")

    # Defining 2nd order DC motor system: G(s) = 5 / (s^2 + 11s + 10)
    Gs = ct.tf([5.0], [1.0, 11.0, 10.0])

    # simulating over t = 5 seconds and 500 data points
    t = np.linspace(0, 5, 500)

    # defining setpoint
    setpoint = np.ones_like(t)

    print(f"System Model G(s) = {Gs}")

    # Plot 1: problem (Uncontrolled System)
    print("\nGenerating Plot 1: Uncontrolled System")
    time_un, res_un, info_un = simulate_pid_with_metrics(0, 0, 0, Gs, t)

    plt.figure(figsize=(10, 6))
    plt.plot(time_un, res_un, label = 'Uncontrolled Motor Speed (G(s))', color='blue', linewidth=2)
    plt.plot(t, setpoint, 'r--', label='Target Speed (Setpoint = 1.0)')
    plt.title('Problem: Uncontrolled motor step response')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed (normalized)')
    plt.grid(True)
    plt.legend()

    # printing uncontrolled error
    print(f"-> Uncontrolled: Ess = {info_un['SteadyStateError']:.2f}")

    # Plot 2: Proportional (P) Tuning
    print("\nGenerating Plot 2: P only Tuning")
    plt.figure(figsize=(10, 6))

    for i in [5, 10, 15]:
        t_p, r_p, info_p = simulate_pid_with_metrics(i, 0, 0, Gs, t)
        plt.plot(t_p, r_p, label=f'Kp = {i} (OS: {info_p["Overshoot"]:.1f}%)')

    plt.plot(t, setpoint, 'r--', label='Setpoint')
    plt.title('Effect of Proportional (Kp) Tuning')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed')
    plt.grid(True)
    plt.legend()

    # Plot 3: Proportional-Derivative (PD) Tuning
    print("\nGenerating Plot 3: PD Tuning")
    plt.figure(figsize=(10, 6))
    Kp_fixed_pd = 10.0 #using good Kp from last step

    for j in [0.5, 1.0, 2.0]:
        t_pd, r_pd, info_pd = simulate_pid_with_metrics(Kp_fixed_pd, 0, j, Gs, t)
        plt.plot(t_pd, r_pd, label=f'Kd = {j} (Kp={Kp_fixed_pd})')

    plt.plot(t, setpoint, 'r--', label='Setpoint (Kp=10)')
    plt.title('Effect of Proportional-Derivative (PD) Tuning')
    plt.xlabel('Time (seconds)')
    plt.ylabel('speed')
    plt.grid(True)
    plt.legend()
    print("PD TUNING REDUCES OVERSHOOT BUT STEADY ERROR REMAINS")

    # Plot 4: Proportioanl-Integral (PI) tuning
    print("\nGenerating Plot 3: PI Tuning")
    plt.figure(figsize=(10, 6))
    Kp_fixed_pi = 10.0 #using a good Kp from P only

    for k in [5, 20, 50]:
        t_pi, r_pi, info_pi = simulate_pid_with_metrics(Kp_fixed_pi, k, 0, Gs, t)
        plt.plot(t_pi, r_pi, label=f'Ki = {k} (OS: {info_pi["Overshoot"]:.1f}%)')

    plt.plot(t, setpoint, 'r--', label='Setpoint (Kp=10)')
    plt.title('Effect of Proportional-Integral (PI) Tuning')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed')
    plt.grid(True)
    plt.legend()

    # Plot 5: Full PID tuning
    print("\nGenerating Plot 5: Full PID tuning")
    plt.figure(figsize=(10, 6))
    Kp_fixed_pid = 10.0
    Ki_fixed_pid = 40.0 # using Ki with significant overshoot

    for l in [0.5, 1.0, 1.5]:
        t_pid, r_pid, info_pid = simulate_pid_with_metrics(Kp_fixed_pid, Ki_fixed_pid, l, Gs, t)
        plt.plot(t_pid, r_pid, label=f'Kd = {l} (OS: {info_pid["Overshoot"]:.1f}%)')
    
    plt.plot(t, setpoint, 'r--', label='Setpoint (Kp=10, Ki=40)')
    plt.title('Effect of Proportional-Integral-Derivative (PID) Tuning')
    plt.xlabel('Time(seconds)')
    plt.ylabel('Speed')
    plt.grid(True)
    plt.legend()
    print("now here D-term successfully damps the oscillations")

    # Plot 6: Final Comparison (manual vs Ziegler-Nichols)
    print("\nGenerating Plot 6: Final Comparison and Metrics")
    plt.figure(figsize=(12, 7))
    print("Performance Metrics Table for README")

    # an uncontrolled system for reference
    plt.plot(time_un, res_un, label='uncontrolled System', linestyle=':', color='gray')
    print(f"Uncontrolled      | Kp=0.0  | Ki=0.0  | Kd=0.0  | OS={info_un['Overshoot']:.1f}% | Ts={info_un['SettlingTime']:.3f}s | Ess={info_un['SteadyStateError']:.3f}")

    # Ziegler-Nichols gains (from seperate analysis)
    # Ku = 24.2, Tu = 0.598
    Kp_zn = 14.52
    Ki_zn = 48.56
    Kd_zn = 1.085
    t_zn, r_zn, info_zn = simulate_pid_with_metrics(Kp_zn, Ki_zn, Kd_zn, Gs, t)
    plt.plot(t_zn, r_zn, label=f'Ziegler-Nichols Tune (OS: {info_zn["Overshoot"]:.1f}%)')
    print(f"Ziegler-Nichols | Kp={Kp_zn:.1f} | Ki={Ki_zn:.1f} | Kd={Kd_zn:.1f} | OS={info_zn['Overshoot']:.1f}% | Ts={info_zn['SettlingTime']:.3f}s | Ess={info_zn['SteadyStateError']:.3f}")

    # Best manual time (example)
    Kp_man = 10.0
    Ki_man = 40.0
    Kd_man = 1.5
    t_man, r_man, info_man = simulate_pid_with_metrics(Kp_man, Ki_man, Kd_man, Gs, t)
    plt.plot(t_man, r_man, label=f'Manual Tune (OS: {info_man["Overshoot"]:.1f}%)', linewidth=2, color='green')
    print(f"Manual Tune       | Kp={Kp_man:.1f} | Ki={Ki_man:.1f} | Kd={Kd_man:.1f} | OS={info_man['Overshoot']:.1f}% | Ts={info_man['SettlingTime']:.3f}s | Ess={info_man['SteadyStateError']:.3f}")

    plt.plot(t, setpoint, 'r--', label='Setpoint')
    plt.title('Final Comparison: Manual Tune vs. Ziegler-Nichols')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Speed')
    plt.grid(True)
    plt.legend()

    # Final Output

    print("All simulations are complete")

    plt.show()