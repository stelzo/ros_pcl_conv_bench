import os
import json
import re
import matplotlib.pyplot as plt

def abbreviate_command(command, usecase):
    command = re.sub(r'\./build/pcl/', r'pcl/', command)
    command = re.sub(r'\./rpcl2/target/release/', r'rpcl2/', command)
    return command

def read_results(directory):
    results = {}
    for filename in os.listdir(directory):
        if filename.endswith(".json"):
            with open(os.path.join(directory, filename), 'r') as file:
                data = json.load(file)
                usecase = filename.split('_')[0]
                results[filename] = {'usecase': usecase, 'results': data['results']}
    return results

def plot_execution_time(results, output_dir):
    for filename, data in results.items():
        plt.figure(figsize=(12, 6))
        plt.title(f'Execution Time - {filename}')
        usecase = data['usecase']
        commands = [abbreviate_command(run['command'], usecase) for run in data['results']]
        execution_times = [run['mean'] * 1000 for run in data['results']]  # Convert to milliseconds
        stddev = [run['stddev'] * 1000 for run in data['results']]  # Convert to milliseconds
        plt.bar(commands, execution_times, yerr=stddev, capsize=5)
        plt.xlabel('Command')
        plt.ylabel('Execution Time (ms)')
        plt.xticks(rotation=45)
        plt.tight_layout()  # Adjust layout dynamically
        plt.savefig(os.path.join(output_dir, f'execution_time_{filename}.png'))
        plt.close()

def plot_user_system_time(results, output_dir):
    for filename, data in results.items():
        plt.figure(figsize=(12, 6))
        plt.title(f'User and System Time - {filename}')
        usecase = data['usecase']
        commands = [abbreviate_command(run['command'], usecase) for run in data['results']]
        user_times = [run['user'] * 1000 for run in data['results']]  # Convert to milliseconds
        system_times = [run['system'] * 1000 for run in data['results']]  # Convert to milliseconds
        x = range(len(commands))
        plt.bar(x, user_times, width=0.4, label='User Time', bottom=0)  # Move bottom to 0
        plt.bar(x, system_times, bottom=user_times, width=0.4, label='System Time')
        plt.xlabel('Command')
        plt.ylabel('Time (ms)')
        plt.xticks(x, commands, rotation=45)
        plt.legend()
        plt.tight_layout()  # Adjust layout dynamically
        plt.savefig(os.path.join(output_dir, f'user_system_time_{filename}.png'))
        plt.close()

if __name__ == "__main__":
    directory = 'results/'
    output_dir = 'images'
    os.makedirs(output_dir, exist_ok=True)
    results = read_results(directory)
    plot_execution_time(results, output_dir)
    plot_user_system_time(results, output_dir)
    print('Plots generated successfully in the images directory.')
