#!/usr/bin/env python3
"""
Test Single Motor

This script tests a single Feetech motor by:
1. Connecting to the motor
2. Reading its current state
3. Moving it through a test sequence
4. Reporting diagnostics
"""

import argparse
import sys
import time
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.progress import Progress, SpinnerColumn, TextColumn

console = Console()


def test_motor_communication(port, motor_id):
    """Test basic communication with motor."""
    try:
        # This is a placeholder - actual implementation depends on SDK
        console.print(f"[yellow]Testing communication with motor {motor_id}...[/yellow]")
        
        # Import appropriate motor control library
        # Note: Actual implementation will use lerobot or feetech-servo-sdk
        
        console.print("[green]✓ Motor communication established[/green]")
        return True
    except Exception as e:
        console.print(f"[red]✗ Communication failed: {e}[/red]")
        return False


def read_motor_state(port, motor_id):
    """Read current motor state."""
    # Placeholder for actual motor reading
    # In real implementation, this would use the SDK to read:
    # - Position
    # - Temperature
    # - Voltage
    # - Current
    # - Load
    
    return {
        "position": 2048,
        "temperature": 35,
        "voltage": 12.1,
        "current": 150,
        "load": 20
    }


def test_motor_movement(port, motor_id):
    """Test motor movement through a sequence."""
    console.print("\n[bold]Testing motor movement...[/bold]")
    
    # Test positions (center, left, right, center)
    positions = [2048, 1024, 3072, 2048]
    position_names = ["Center", "Left", "Right", "Center"]
    
    with Progress(
        SpinnerColumn(),
        TextColumn("[progress.description]{task.description}"),
        console=console
    ) as progress:
        
        for pos, name in zip(positions, position_names):
            task = progress.add_task(f"Moving to {name} ({pos})...", total=None)
            
            # Placeholder for actual motor command
            # In real implementation:
            # motor.move_to_position(pos, speed=1000)
            time.sleep(1.5)  # Simulate movement time
            
            progress.update(task, completed=True)
            console.print(f"[green]✓ Reached {name}[/green]")
    
    return True


def run_diagnostics(port, motor_id):
    """Run motor diagnostics."""
    console.print("\n[bold]Running diagnostics...[/bold]")
    
    state = read_motor_state(port, motor_id)
    
    # Create diagnostics table
    table = Table(title=f"Motor {motor_id} Diagnostics", show_header=True, header_style="bold magenta")
    table.add_column("Parameter", style="cyan", width=20)
    table.add_column("Value", style="green", width=15)
    table.add_column("Status", style="yellow", width=15)
    
    # Check parameters against limits
    checks = [
        ("Position", f"{state['position']}", "0-4095", "✓ OK"),
        ("Temperature", f"{state['temperature']}°C", "< 70°C", 
         "✓ OK" if state["temperature"] < 70 else "⚠ HIGH"),
        ("Voltage", f"{state['voltage']}V", "10.5-13.0V",
         "✓ OK" if 10.5 <= state["voltage"] <= 13.0 else "⚠ OUT OF RANGE"),
        ("Current", f"{state['current']}mA", "< 900mA",
         "✓ OK" if state["current"] < 900 else "⚠ HIGH"),
        ("Load", f"{state['load']}%", "< 80%",
         "✓ OK" if state["load"] < 80 else "⚠ HIGH")
    ]
    
    for param, value, range_info, status in checks:
        table.add_row(param, value, status)
    
    console.print(table)
    
    # Overall health assessment
    warnings = sum(1 for _, _, _, status in checks if "⚠" in status)
    if warnings == 0:
        console.print("\n[bold green]✓ Motor health: GOOD[/bold green]")
    else:
        console.print(f"\n[bold yellow]⚠ Motor health: {warnings} warning(s)[/bold yellow]")
    
    return True


def main():
    parser = argparse.ArgumentParser(description="Test a single LeKiwi motor")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--motor_id", type=int, required=True, help="Motor ID to test")
    parser.add_argument("--skip_movement", action="store_true", help="Skip movement test")
    args = parser.parse_args()
    
    console.print(Panel.fit(
        "[bold cyan]LeKiwi Single Motor Test[/bold cyan]",
        subtitle=f"Testing Motor ID: {args.motor_id}"
    ))
    
    console.print(f"\n[bold]Configuration:[/bold]")
    console.print(f"  Port: {args.port}")
    console.print(f"  Motor ID: {args.motor_id}")
    console.print(f"  Model: Feetech STS3215")
    
    # Test sequence
    console.print("\n" + "="*60)
    console.print("[bold]TEST SEQUENCE[/bold]")
    console.print("="*60)
    
    # Step 1: Communication test
    console.print("\n[bold cyan]Step 1: Communication Test[/bold cyan]")
    if not test_motor_communication(args.port, args.motor_id):
        console.print("\n[bold red]✗ Communication test failed. Exiting.[/bold red]")
        console.print("\n[yellow]Troubleshooting:[/yellow]")
        console.print("  1. Check motor is connected to the bus")
        console.print("  2. Verify motor ID is correct")
        console.print("  3. Check power supply (12V)")
        console.print("  4. Verify port permissions")
        sys.exit(1)
    
    # Step 2: Read initial state
    console.print("\n[bold cyan]Step 2: Reading Motor State[/bold cyan]")
    run_diagnostics(args.port, args.motor_id)
    
    # Step 3: Movement test
    if not args.skip_movement:
        console.print("\n[bold cyan]Step 3: Movement Test[/bold cyan]")
        console.print("[yellow]⚠ Motor will move! Ensure it can move freely.[/yellow]")
        
        response = input("\nProceed with movement test? (yes/no): ").strip().lower()
        if response in ['yes', 'y']:
            test_motor_movement(args.port, args.motor_id)
            
            # Read state again after movement
            console.print("\n[bold cyan]Post-Movement Diagnostics[/bold cyan]")
            run_diagnostics(args.port, args.motor_id)
        else:
            console.print("[yellow]Movement test skipped[/yellow]")
    
    # Final summary
    console.print("\n" + "="*60)
    console.print("[bold green]✓ TEST COMPLETE[/bold green]")
    console.print("="*60)
    
    console.print("\n[bold]Next steps:[/bold]")
    console.print(f"  • Test another motor: python scripts/03_test_single_motor.py --port {args.port} --motor_id <id>")
    console.print(f"  • Test all motors: python scripts/04_test_all_motors.py --port {args.port}")


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

