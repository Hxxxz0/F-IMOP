"""
URDF Support Test Script with Visualization
============================================

This script tests the URDFModel functionality and generates a visualization
of tracking performance and F-IMOP intervention.

Usage:
    python examples/urdf_test.py
"""

import sys
import os
import numpy as np
import matplotlib.pyplot as plt

# Add package path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import pinocchio
except ImportError:
    print("Error: 'pinocchio' library not found.")
    print("Please install it: pip install pin")
    sys.exit(1)

from fimop import FIMOPController
from fimop.dynamics import URDFModel
from fimop.utils import generate_figure8_trajectory


def run_urdf_test():
    """Run URDF loading and simulation test with visualization."""
    
    # ==========================================
    # 1. Load URDF Model
    # ==========================================
    urdf_path = os.path.join(os.path.dirname(__file__), '..', 'assets', 'simple_arm.urdf')
    urdf_path = os.path.abspath(urdf_path)
    
    print("=" * 60)
    print("F-IMOP URDF Support Test")
    print("=" * 60)
    print(f"\nLoading URDF: {urdf_path}")
    
    try:
        robot = URDFModel(urdf_path)
    except Exception as e:
        print(f"Failed to load URDF: {e}")
        return

    print(f"✓ Robot loaded: {robot.dof} DOF")
    
    # ==========================================
    # 2. Initialize F-IMOP Controller
    # ==========================================
    fimop = FIMOPController(robot, Lambda=5.0, decay_rate=3.0)
    print(f"✓ Controller initialized: {fimop}")
    
    # ==========================================
    # 3. Simulation Setup
    # ==========================================
    dt = 0.002
    duration = 3.0
    steps = int(duration / dt)
    
    q = np.zeros(robot.dof)
    dq = np.zeros(robot.dof)
    
    get_ref = generate_figure8_trajectory(robot.dof, frequency=1.0, amplitude=0.8)
    
    # Data logging
    log_t = []
    log_err = []
    log_V = []
    log_correction = []
    log_q = []
    log_q_d = []
    
    print(f"\n✓ Running simulation: {duration}s @ {1/dt:.0f}Hz...")
    
    # ==========================================
    # 4. Control Loop
    # ==========================================
    for i in range(steps):
        t = i * dt
        q_d, dq_d, ddq_d = get_ref(t)
        
        # Nominal controller: simple PD
        Kp, Kd = 50.0, 10.0
        tau_nom = Kp * (q_d - q) + Kd * (dq_d - dq)
        
        # F-IMOP Safety Filter
        tau_safe, info = fimop.compute_safe_control(q, dq, q_d, dq_d, ddq_d, tau_nom)
        
        # Forward dynamics
        ddq = robot.forward_dynamics(q, dq, tau_safe)
        dq = dq + ddq * dt
        q = q + dq * dt
        
        # Log
        log_t.append(t)
        log_err.append(np.linalg.norm(q - q_d))
        log_V.append(info['V'])
        log_correction.append(info['correction'])
        log_q.append(q.copy())
        log_q_d.append(q_d.copy())
    
    # Convert to arrays
    log_t = np.array(log_t)
    log_err = np.array(log_err)
    log_V = np.array(log_V)
    log_correction = np.array(log_correction)
    log_q = np.array(log_q)
    log_q_d = np.array(log_q_d)
    
    # ==========================================
    # 5. Print Statistics
    # ==========================================
    state = fimop.get_filter_state()
    print(f"\n✓ Simulation completed!")
    print(f"  RMSE: {np.sqrt((log_err**2).mean()):.6f} rad")
    print(f"  Max Error: {log_err.max():.6f} rad")
    print(f"  F-IMOP Activation: {state['activation_ratio']*100:.1f}%")
    
    # ==========================================
    # 6. Visualization
    # ==========================================
    print("\n✓ Generating plots...")
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("F-IMOP with URDF Model: Simulation Results", fontsize=14, fontweight='bold')
    
    # Plot 1: Joint Trajectories
    ax1 = axes[0, 0]
    for j in range(robot.dof):
        ax1.plot(log_t, log_q[:, j], label=f'q{j+1} (actual)', linewidth=1.5)
        ax1.plot(log_t, log_q_d[:, j], '--', label=f'q{j+1} (target)', alpha=0.7)
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Joint Angle (rad)')
    ax1.set_title('Joint Trajectories')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # Plot 2: Tracking Error
    ax2 = axes[0, 1]
    ax2.plot(log_t, log_err, 'r-', linewidth=1.5)
    ax2.fill_between(log_t, 0, log_err, color='red', alpha=0.2)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Error Norm (rad)')
    ax2.set_title('Tracking Error')
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Lyapunov Function
    ax3 = axes[1, 0]
    ax3.semilogy(log_t, log_V + 1e-10, 'b-', linewidth=1.5)  # +eps to avoid log(0)
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('V (log scale)')
    ax3.set_title('Lyapunov Function Decay')
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: F-IMOP Correction
    ax4 = axes[1, 1]
    ax4.plot(log_t, log_correction, 'g-', linewidth=1.5)
    ax4.fill_between(log_t, 0, log_correction, color='green', alpha=0.2)
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Correction (Nm)')
    ax4.set_title('F-IMOP Intervention')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save figure
    output_path = os.path.join(os.path.dirname(__file__), '..', 'results', 'Fig_URDF_Test.png')
    output_path = os.path.abspath(output_path)
    plt.savefig(output_path, dpi=150)
    print(f"\n✓ Figure saved: {output_path}")
    
    # Show plot
    plt.show()
    
    print("\n" + "=" * 60)
    print("URDF Test Completed Successfully! ✅")
    print("=" * 60)


if __name__ == "__main__":
    run_urdf_test()
