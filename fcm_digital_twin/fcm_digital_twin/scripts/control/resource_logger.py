#!/usr/bin/env python3
import psutil
import time
import csv
from datetime import datetime

PROCESS_NAMES = ['slam_toolbox', 'planner_server', 'controller_server']

def get_process_metrics():
    metrics = {'cpu': 0.0, 'ram_mb': 0.0}
    for proc in psutil.process_iter(['name', 'cpu_percent', 'memory_info']):
        try:
            if any(name in proc.info['name'] for name in PROCESS_NAMES):
                metrics['cpu'] += proc.cpu_percent(interval=1.0)
                metrics['ram_mb'] += proc.info['memory_info'].rss / (1024 * 1024)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    return metrics

def main():
    csv_file = f"resource_log_{datetime.now().strftime('%H-%M-%S')}.csv"
    with open(csv_file, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time_s', 'CPU_Percent', 'RAM_MB'])
        
        print(f"[{datetime.now().strftime('%H:%M:%S')}] Starting logging in {csv_file}...")
        start_time = time.time()
        
        try:
            while True:
                get_process_metrics()
                time.sleep(1.0)
                
                metrics = get_process_metrics()
                elapsed_time = round(time.time() - start_time, 1)
                
                writer.writerow([elapsed_time, metrics['cpu'], metrics['ram_mb']])
                print(f"Time: {elapsed_time}s | CPU: {metrics['cpu']:.1f}% | RAM: {metrics['ram_mb']:.1f} MB", end='\r')
                
        except KeyboardInterrupt:
            print("\nLogging completed.")

if __name__ == '__main__':
    main()