#!/usr/bin/env python3
"""
Find USB Port for LeKiwi Motor Controller

This script helps identify the correct USB port for the motor controller
by listing all available serial ports.
"""

import sys
import serial.tools.list_ports
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

console = Console()


def find_ports():
    """List all available serial ports."""
    ports = list(serial.tools.list_ports.comports())
    return ports


def main():
    console.print(Panel.fit(
        "[bold cyan]LeKiwi Motor Controller Port Finder[/bold cyan]",
        subtitle="Step 1: Identify USB Port"
    ))
    
    console.print("\n[yellow]Searching for serial ports...[/yellow]\n")
    
    ports = find_ports()
    
    if not ports:
        console.print("[bold red]✗ No serial ports found![/bold red]")
        console.print("\nTroubleshooting:")
        console.print("  1. Ensure the motor controller is connected via USB")
        console.print("  2. Check if the USB cable is functioning")
        console.print("  3. Try a different USB port on your computer")
        console.print("  4. Check if you have permission to access serial ports:")
        console.print("     [cyan]groups | grep dialout[/cyan]")
        sys.exit(1)
    
    # Create table
    table = Table(title="Available Serial Ports", show_header=True, header_style="bold magenta")
    table.add_column("Port", style="cyan", width=20)
    table.add_column("Description", style="green")
    table.add_column("Hardware ID", style="yellow")
    
    for port in ports:
        table.add_row(
            port.device,
            port.description or "N/A",
            port.hwid or "N/A"
        )
    
    console.print(table)
    
    # Provide guidance
    console.print("\n[bold green]✓ Found {} port(s)[/bold green]".format(len(ports)))
    console.print("\n[bold]How to identify your motor controller:[/bold]")
    console.print("  1. Note the ports listed above")
    console.print("  2. Disconnect the motor controller USB cable")
    console.print("  3. Run this script again")
    console.print("  4. The missing port is your motor controller")
    console.print("\n[bold]Common port names:[/bold]")
    console.print("  • Linux: /dev/ttyUSB0, /dev/ttyACM0")
    console.print("  • macOS: /dev/tty.usbmodem*, /dev/tty.usbserial*")
    console.print("  • Windows: COM3, COM4, etc.")
    
    # If only one port, highlight it
    if len(ports) == 1:
        console.print(f"\n[bold green]→ Only one port detected: {ports[0].device}[/bold green]")
        console.print("  This is likely your motor controller!")
        console.print(f"\n[bold]Next step:[/bold]")
        console.print(f"  python scripts/02_configure_motors.py --port {ports[0].device}")
    else:
        console.print(f"\n[bold]Next step (after identifying port):[/bold]")
        console.print("  python scripts/02_configure_motors.py --port /dev/ttyUSB0")
    
    console.print("\n[dim]Tip: If you get 'Permission denied', run:[/dim]")
    console.print("[dim]  sudo chmod 666 /dev/ttyUSB0[/dim]")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n\n[yellow]Interrupted by user[/yellow]")
        sys.exit(0)
    except Exception as e:
        console.print(f"\n[bold red]Error: {e}[/bold red]")
        sys.exit(1)

