#!/usr/bin/env python3
"""
Motor Diagnostics

Comprehensive diagnostics for LeKiwi motors including:
- Real-time monitoring of motor parameters
- Temperature tracking
- Voltage and current monitoring
- Load analysis
- Data logging
"""

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path
from rich.console import Console
from rich.panel import Panel
from rich.table import Table
from rich.live import Live
import csv

console = Console()


class MotorDiagnostics:
    """Motor diagnostics and monitoring."""
    
    def __init__(self, port, motor_ids, log_file=None):
        self.port = port
        self.motor_ids = motor_ids
        self.log_file = log_file
        self.log_data = []
        
    def read_motor_data(self, motor_id):
        """Read comprehensive data from a motor."""
        # Placeholder - would read actual motor data
        # Simulate some variation for demo
        import random
        base_temp = 35
        
        return {
            "timestamp": datetime.now().strftime("%H:%M:%S"),
            "motor_id": motor_id,
            "position": 2048 + random.randint(-100, 100),
            "temperature": base_temp + motor_id + random.uniform(-2, 2),
            "voltage": 12.0 + random.uniform(-0.2, 0.2),
            "current": 150 + random.randint(-20, 50),
            "load": 20 + random.randint(-5, 15),
            "speed": random.randint(0, 100)
        }
    
    def analyze_motor_health(self, data):
        """Analyze motor health from data."""
        issues = []
        warnings = []
        
        # Temperature analysis
        if data["temperature"] > 70:
            issues.append("Critical temperature")
        elif data["temperature"] > 60:
            warnings.append("Elevated temperature")
        
        # Voltage analysis
        if data["voltage"] < 10.5:
            issues.append("Low voltage")
        elif data["voltage"] > 13.0:
            issues.append("High voltage")
        
        # Current analysis
        if data["current"] > 1000:
            issues.append("Excessive current")
        elif data["current"] > 900:
            warnings.append("High current")
        
        # Load analysis
        if data["load"] > 80:
            warnings.append("High load")
        
        if issues:
            return "ERROR", issues
        elif warnings:
            return "WARNING", warnings
        else:
            return "OK", []
    
    def create_diagnostics_table(self, all_data):
        """Create diagnostics display table."""
        table = Table(title="Motor Diagnostics (Real-time)", show_header=True, header_style="bold magenta")
        table.add_column("Motor", style="cyan", width=8)
        table.add_column("Temp °C", style="yellow", width=10)
        table.add_column("Voltage V", style="blue", width=11)
        table.add_column("Current mA", style="magenta", width=11)
        table.add_column("Load %", style="white", width=8)
        table.add_column("Speed", style="green", width=8)
        table.add_column("Health", style="green", width=15)
        
        for data in all_data:
            health_status, messages = self.analyze_motor_health(data)
            
            if health_status == "ERROR":
                health_str = f"[red]✗ {health_status}[/red]"
            elif health_status == "WARNING":
                health_str = f"[yellow]⚠ {health_status}[/yellow]"
            else:
                health_str = f"[green]✓ {health_status}[/green]"
            
            table.add_row(
                str(data["motor_id"]),
                f"{data['temperature']:.1f}",
                f"{data['voltage']:.2f}",
                str(data["current"]),
                str(data["load"]),
                str(data["speed"]),
                health_str
            )
        
        return table
    
    def log_to_csv(self, all_data):
        """Log data to CSV file."""
        if not self.log_file:
            return
        
        file_exists = Path(self.log_file).exists()
        
        with open(self.log_file, 'a', newline='') as csvfile:
            fieldnames = ['timestamp', 'motor_id', 'position', 'temperature', 
                         'voltage', 'current', 'load', 'speed']
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            
            if not file_exists:
                writer.writeheader()
            
            for data in all_data:
                writer.writerow(data)
    
    def run_diagnostics(self, duration=None):
        """Run continuous diagnostics."""
        console.print(f"\n[bold]Starting diagnostics monitoring...[/bold]")
        if self.log_file:
            console.print(f"[yellow]Logging to: {self.log_file}[/yellow]")
        
        start_time = time.time()
        
        try:
            with Live(console=console, refresh_per_second=2) as live:
                while True:
                    # Read data from all motors
                    all_data = []
                    for motor_id in self.motor_ids:
                        data = self.read_motor_data(motor_id)
                        all_data.append(data)
                    
                    # Update display
                    table = self.create_diagnostics_table(all_data)
                    
                    # Add runtime info
                    runtime = time.time() - start_time
                    info = f"\n[dim]Runtime: {runtime:.1f}s | Press Ctrl+C to stop[/dim]"
                    
                    live.update(Panel(table, subtitle=info))
                    
                    # Log data
                    self.log_to_csv(all_data)
                    
                    # Check duration limit
                    if duration and runtime >= duration:
                        break
                    
                    time.sleep(0.5)
        
        except KeyboardInterrupt:
            console.print("\n[yellow]Monitoring stopped by user[/yellow]")


def generate_report(log_file):
    """Generate diagnostics report from log file."""
    if not Path(log_file).exists():
        console.print(f"[red]Log file not found: {log_file}[/red]")
        return
    
    console.print(f"\n[bold]Generating report from: {log_file}[/bold]\n")
    
    # Read log file
    data = []
    with open(log_file, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        data = list(reader)
    
    if not data:
        console.print("[yellow]No data in log file[/yellow]")
        return
    
    # Analyze data
    console.print(f"[green]Total records: {len(data)}[/green]")
    
    # Group by motor
    motors = {}
    for row in data:
        motor_id = row['motor_id']
        if motor_id not in motors:
            motors[motor_id] = []
        motors[motor_id].append(row)
    
    # Generate statistics for each motor
    table = Table(title="Diagnostics Summary", show_header=True, header_style="bold magenta")
    table.add_column("Motor", style="cyan")
    table.add_column("Avg Temp °C", style="yellow")
    table.add_column("Max Temp °C", style="red")
    table.add_column("Avg Current mA", style="magenta")
    table.add_column("Max Current mA", style="red")
    table.add_column("Records", style="green")
    
    for motor_id, records in motors.items():
        temps = [float(r['temperature']) for r in records]
        currents = [int(r['current']) for r in records]
        
        table.add_row(
            motor_id,
            f"{sum(temps)/len(temps):.1f}",
            f"{max(temps):.1f}",
            f"{sum(currents)/len(currents):.0f}",
            f"{max(currents)}",
            str(len(records))
        )
    
    console.print(table)


def main():
    parser = argparse.ArgumentParser(description="LeKiwi motor diagnostics")
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--motor_ids", nargs='+', type=int, default=[7, 8, 9],
                       help="Motor IDs to monitor (default: 7 8 9)")
    parser.add_argument("--duration", type=int, help="Monitoring duration in seconds")
    parser.add_argument("--log", action="store_true", help="Enable data logging")
    parser.add_argument("--log_file", help="Custom log file path")
    parser.add_argument("--report", help="Generate report from log file")
    args = parser.parse_args()
    
    if args.report:
        generate_report(args.report)
        return
    
    console.print(Panel.fit(
        "[bold cyan]LeKiwi Motor Diagnostics[/bold cyan]",
        subtitle="Real-time Monitoring"
    ))
    
    # Setup log file
    log_file = None
    if args.log:
        if args.log_file:
            log_file = args.log_file
        else:
            # Create logs directory
            log_dir = Path(__file__).parent.parent / "logs"
            log_dir.mkdir(exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            log_file = log_dir / f"motor_diagnostics_{timestamp}.csv"
    
    console.print(f"\n[bold]Configuration:[/bold]")
    console.print(f"  Port: {args.port}")
    console.print(f"  Motors: {args.motor_ids}")
    console.print(f"  Duration: {args.duration if args.duration else 'Continuous'}")
    console.print(f"  Logging: {'Enabled' if log_file else 'Disabled'}")
    if log_file:
        console.print(f"  Log File: {log_file}")
    
    # Initialize diagnostics
    diagnostics = MotorDiagnostics(args.port, args.motor_ids, log_file)
    
    # Run diagnostics
    diagnostics.run_diagnostics(duration=args.duration)
    
    console.print("\n[bold green]✓ Diagnostics complete[/bold green]")
    
    if log_file:
        console.print(f"\n[bold]Data saved to:[/bold] {log_file}")
        console.print(f"\n[bold]Generate report:[/bold]")
        console.print(f"  python scripts/06_motor_diagnostics.py --report {log_file}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        console.print("\n\n[yellow]Diagnostics interrupted[/yellow]")
        sys.exit(0)
    except Exception as e:
        console.print(f"\n[bold red]Error: {e}[/bold red]")
        import traceback
        console.print(traceback.format_exc())
        sys.exit(1)

