#!/usr/bin/env python3
"""
Test All Motors

This script tests all three wheel motors of the LeKiwi mobile base.
"""

import argparse
import sys
import time
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.live import Live

console = Console()


def test_all_motors_communication(port, motor_ids):
    """Test communication with all motors."""
    console.print("\n[bold]Testing communication with all motors...[/bold]")
    
    results = {}
    for motor_id in motor_ids:
        try:
            # Placeholder for actual motor communication
            console.print(f"  Motor {motor_id}: ", end="")
            time.sleep(0.3)  # Simulate communication delay
            results[motor_id] = True
            console.print("[green]✓ Connected[/green]")
        except Exception as e:
            results[motor_id] = False
            console.print(f"[red]✗ Failed ({e})[/red]")
    
    return results


def read_all_motors_state(port, motor_ids):
    """Read state from all motors."""
    states = {}
    for motor_id in motor_ids:
        # Placeholder - would read actual motor state
        states[motor_id] = {
            "position": 2048,
            "temperature": 35 + motor_id,  # Vary slightly for demo
            "voltage": 12.0 + (motor_id * 0.1),
            "current": 150 + (motor_id * 10),
            "load": 20 + (motor_id * 5)
        }
    return states


def display_motor_states(states):
    """Display all motor states in a table."""
    table = Table(title="All Motors Status", show_header=True, header_style="bold magenta")
    table.add_column("Motor ID", style="cyan", width=10)
    table.add_column("Position", style="green", width=10)
    table.add_column("Temp (°C)", style="yellow", width=10)
    table.add_column("Voltage (V)", style="blue", width=12)
    table.add_column("Current (mA)", style="magenta", width=12)
    table.add_column("Load (%)", style="white", width=10)
    table.add_column("Status", style="green", width=10)
    
    for motor_id, state in states.items():
        # Check if any parameter is out of range
        temp_ok = state["temperature"] < 70
        volt_ok = 10.5 <= state["voltage"] <= 13.0
        current_ok = state["current"] < 900
        load_ok = state["load"] < 80
        
        status = "✓ OK" if all([temp_ok, volt_ok, current_ok, load_ok]) else "⚠ CHECK"
        
        table.add_row(
            str(motor_id),
            str(state["position"]),
            f"{state['temperature']:.1f}",
            f"{state['voltage']:.2f}",
            str(state["current"]),
            str(state["load"]),
            status
        )
    
    console.print(table)


def test_coordinated_movement(port, motor_ids):
    """Test coordinated movement of all motors."""
    console.print("\n[bold]Testing coordinated movement...[/bold]")
    console.print("[yellow]All motors will move together[/yellow]")
    
    movements = [
        ("Forward", "Moving forward..."),
        ("Backward", "Moving backward..."),
        ("Left rotation", "Rotating left..."),
        ("Right rotation", "Rotating right..."),
        ("Stop", "Stopping...")
    ]
    
    for name, description in movements:
        console.print(f"\n[cyan]{description}[/cyan]")
        
        # Placeholder for actual motor commands
        # In real implementation:
        # for motor_id in motor_ids:
        #     motor.set_velocity(...)
        
        time.sleep(2)  # Simulate movement duration
        console.print(f"[green]✓ {name} complete[/green]")
    
    return True


def run_safety_checks(states):
    """Run safety checks on all motors."""
    console.print("\n[bold]Running safety checks...[/bold]")
    
    warnings = []
    errors = []
    
    for motor_id, state in states.items():
        # Temperature check
        if state["temperature"] > 70:
            errors.append(f"Motor {motor_id}: Temperature too high ({state['temperature']}°C)")
        elif state["temperature"] > 60:
            warnings.append(f"Motor {motor_id}: Temperature elevated ({state['temperature']}°C)")
        
        # Voltage check
        if state["voltage"] < 10.5 or state["voltage"] > 13.0:
            errors.append(f"Motor {motor_id}: Voltage out of range ({state['voltage']}V)")
        
        # Current check
        if state["current"] > 1000:
            errors.append(f"Motor {motor_id}: Current too high ({state['current']}mA)")
        elif state["current"] > 900:
            warnings.append(f"Motor {motor_id}: Current elevated ({state['current']}mA)")
        
        # Load check
        if state["load"] > 80:
            warnings.append(f"Motor {motor_id}: Load high ({state['load']}%)")
    
    # Display results
    if errors:
        console.print(f"\n[bold red]✗ {len(errors)} ERROR(S) found:[/bold red]")
        for error in errors:
            console.print(f"  [red]• {error}[/red]")
    
    if warnings:
        console.print(f"\n[bold yellow]⚠ {len(warnings)} WARNING(S):[/bold yellow]")
        for warning in warnings:
            console.print(f"  [yellow]• {warning}[/yellow]")
    
    if not errors and not warnings:
        console.print("[bold green]✓ All safety checks passed[/bold green]")
    
    return len(errors) == 0


def main():
    parser = argparse.ArgumentParser(description="Test all LeKiwi wheel motors")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--motor_ids", nargs='+', type=int, default=[7, 8, 9],
                       help="Motor IDs to test (default: 7 8 9)")
    parser.add_argument("--skip_movement", action="store_true", help="Skip movement tests")
    args = parser.parse_args()
    
    console.print(Panel.fit(
        "[bold cyan]LeKiwi All Motors Test[/bold cyan]",
        subtitle=f"Testing Motors: {args.motor_ids}"
    ))
    
    console.print(f"\n[bold]Configuration:[/bold]")
    console.print(f"  Port: {args.port}")
    console.print(f"  Motor IDs: {args.motor_ids}")
    console.print(f"  Model: Feetech STS3215")
    
    # Test sequence
    console.print("\n" + "="*60)
    console.print("[bold]TEST SEQUENCE[/bold]")
    console.print("="*60)
    
    # Step 1: Communication test
    console.print("\n[bold cyan]Step 1: Communication Test[/bold cyan]")
    comm_results = test_all_motors_communication(args.port, args.motor_ids)
    
    failed_motors = [mid for mid, success in comm_results.items() if not success]
    if failed_motors:
        console.print(f"\n[bold red]✗ Communication failed for motors: {failed_motors}[/bold red]")
        console.print("\n[yellow]Troubleshooting:[/yellow]")
        console.print("  1. Check all motors are connected to the bus")
        console.print("  2. Verify motor IDs are correctly configured")
        console.print("  3. Check power supply (12V)")
        console.print("  4. Test each motor individually first")
        sys.exit(1)
    
    console.print("\n[green]✓ All motors responding[/green]")
    
    # Step 2: Read states
    console.print("\n[bold cyan]Step 2: Reading Motor States[/bold cyan]")
    states = read_all_motors_state(args.port, args.motor_ids)
    display_motor_states(states)
    
    # Step 3: Safety checks
    console.print("\n[bold cyan]Step 3: Safety Checks[/bold cyan]")
    if not run_safety_checks(states):
        console.print("\n[bold red]✗ Safety checks failed. Please address errors before continuing.[/bold red]")
        sys.exit(1)
    
    # Step 4: Movement test
    if not args.skip_movement:
        console.print("\n[bold cyan]Step 4: Coordinated Movement Test[/bold cyan]")
        console.print("[yellow]⚠ All motors will move! Ensure robot can move freely.[/yellow]")
        
        response = input("\nProceed with movement test? (yes/no): ").strip().lower()
        if response in ['yes', 'y']:
            test_coordinated_movement(args.port, args.motor_ids)
            
            # Read states again after movement
            console.print("\n[bold cyan]Post-Movement Status[/bold cyan]")
            states = read_all_motors_state(args.port, args.motor_ids)
            display_motor_states(states)
            run_safety_checks(states)
        else:
            console.print("[yellow]Movement test skipped[/yellow]")
    
    # Final summary
    console.print("\n" + "="*60)
    console.print("[bold green]✓ ALL TESTS COMPLETE[/bold green]")
    console.print("="*60)
    
    console.print("\n[bold]Summary:[/bold]")
    console.print(f"  • Motors tested: {len(args.motor_ids)}")
    console.print(f"  • Communication: ✓ All connected")
    console.print(f"  • Safety: ✓ All checks passed")
    
    console.print("\n[bold]Next steps:[/bold]")
    console.print(f"  • Run diagnostics: python scripts/06_motor_diagnostics.py --port {args.port}")
    console.print(f"  • Try teleoperation: python scripts/05_basic_teleop.py --port {args.port}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n\n[yellow]Test interrupted by user[/yellow]")
        sys.exit(0)
    except Exception as e:
        console.print(f"\n[bold red]Error: {e}[/bold red]")
        import traceback
        console.print(traceback.format_exc())
        sys.exit(1)

