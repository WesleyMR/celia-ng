import { Component } from '@angular/core';
import { HttpClient } from '@angular/common/http';
import { MatCardModule } from '@angular/material/card';
import { HttpClientModule } from '@angular/common/http';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { MatButtonModule } from '@angular/material/button';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { MatIconModule } from '@angular/material/icon';
import { MatDividerModule } from '@angular/material/divider';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatSelectModule } from '@angular/material/select';
import { MatInputModule } from '@angular/material/input';

@Component({
  selector: 'app-celia-gazebo',
  standalone: true,
  imports: [
    CommonModule,
    FormsModule,
    HttpClientModule,
    MatCardModule,
    MatButtonModule,
    MatSlideToggleModule,
    MatIconModule,
    MatDividerModule,
    MatFormFieldModule,
    MatSelectModule,
    MatInputModule
  ],
  templateUrl: './celia-gazebo.component.html',
  styleUrl: './celia-gazebo.component.less'
})
export class CeliaGazeboComponent {
  output = '';
  isRunning = false;
  currentCommand: string | null = null;
  showCommand = true;
  showManualInput = false;
  manualCommand = '';
  commandHistory: string[] = [];
  basicCommands = [
    { label: 'Node List', command: 'ros2 node list' },
    { label: 'Topic List', command: 'ros2 topic list' },
    { label: 'Service List', command: 'ros2 service list' },
    { label: 'Param List', command: 'ros2 param list' },
    { label: 'Interfaces List', command: 'ros2 interface list' }
  ];

  paramCommands = [
    { label: 'Node Info', template: 'ros2 node info ' },
    { label: 'Echo Topic', template: 'ros2 topic echo ' },
    { label: 'Topic Type', template: 'ros2 topic type ' },
    { label: 'Service Type', template: 'ros2 service type ' }
  ];

  selectedBasic = '';
  selectedParam = '';
  paramValue = '';
  driverRunning = false;

  constructor(private http: HttpClient) { }

  onBasicSelect(cmd: string) {
    if (!cmd) return;
    this.manualCommand = cmd;
    this.runManualCommand();
    this.selectedBasic = '';
  }

  runParamCommand() {
    if (!this.selectedParam || !this.paramValue.trim()) return;

    if(this.selectedParam == "ros2 topic echo ") {
      this.runEchoTimed();
    } else {
      const cmd = this.selectedParam + this.paramValue.trim();
      this.manualCommand = cmd;
      this.runManualCommand();
      this.paramValue = '';
    }
  }

  runEchoTimed() {
    if (!this.paramValue.trim()) return;
  
    this.output = '';
    this.isRunning = true;
  
    const payload = {
      topic: this.paramValue.trim(),
      duration: 3000 // 3 segundos
    };
  
    if (this.showCommand) {
      this.output += `$ ros2 topic echo ${payload.topic} (por 3s)\n`;
    }
  
    this.http.post<{ output: string }>('http://localhost:12345/ros/echo-timed', payload).subscribe({
      next: (res) => {
        this.output += res.output;
        this.commandHistory.push(`ros2 topic echo ${payload.topic} (3s)`);
        this.isRunning = false;
      },
      error: (err) => {
        this.output += `\nErro: ${err.error?.error || err.message}`;
        this.isRunning = false;
      }
    });
  }
  

  runCommand(command: 'build' | 'packages' | 'driver' | 'keyboard') {
    this.isRunning = true;
    this.currentCommand = command;
    this.output = '';

    let endpoint = '';
    let label = '';

    switch (command) {
      case 'build':
        endpoint = '/ros/build';
        label = 'cd /home/wesley/Documentos/Projetos/tello_ros_ws && source /opt/ros/foxy/setup.bash && colcon build && source cd /home/wesley/Documentos/Projetos/tello_ros_ws/install/setup.bash';
        break;
      case 'packages':
        endpoint = '/ros/tello-packages';
        label = 'ros2 pkg list | grep tello';
        break;
      // case 'driver':
      //   endpoint = '/ros/tello-driver';
      //   label = 'ros2 run tello_py_driver tello_ros2_driver.py';
      //   break;
      case 'keyboard':
        endpoint = '/ros/tello-keyboard';
        label = 'ros2 run teleop_twist_keyboard teleop_twist_keyboard';
        break;
    }

    if (this.showCommand) {
      this.output += `\n$ ${label}\n`;
    }

    this.http.post<{ output: string }>(`http://localhost:12345${endpoint}`, {}).subscribe({
      next: (res) => {
        this.output += res.output;
        this.isRunning = false;
        this.currentCommand = null;
      },
      error: (err) => {
        this.output += `\nErro: ${err.error?.error || err.message}`;
        this.isRunning = false;
        this.currentCommand = null;
      }
    });

    this.commandHistory.push(label);
  }

  toggleDriver() {
    const endpoint = this.driverRunning ? '/ros/stop-driver' : '/ros/tello-driver-v2';
  
    this.http.post<{ output: string }>(`http://localhost:12345${endpoint}`, {}).subscribe({
      next: (res) => {
        this.driverRunning = !this.driverRunning;
        this.output += `\n${res.output}`;
      },
      error: (err) => {
        this.output += `\nErro: ${err.error?.error || err.message}`;
      }
    });
  }

  runManualCommand() {
    if (!this.manualCommand.trim()) return;

    this.output = '';
    this.isRunning = true;

    if (this.showCommand) {
      this.output += `\n$ ${this.manualCommand}\n`;
    }

    this.http.post<{ output: string }>('http://localhost:12345/ros/manual', {
      command: this.manualCommand
    }).subscribe({
      next: (res) => {
        this.output += res.output;
        this.isRunning = false;
      },
      error: (err) => {
        this.output += `\nErro: ${err.error?.error || err.message}`;
        this.isRunning = false;
      }
    });
    this.commandHistory.push(this.manualCommand);
  }

  runFromHistory(cmd: string) {
    this.manualCommand = cmd;
    this.runManualCommand();
  }
}