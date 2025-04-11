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
  selector: 'app-celia-run',
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
  templateUrl: './celia-run.component.html',
  styleUrl: './celia-run.component.less'
})
export class CeliaRunComponent {
  showTestFlight = false;
  isRunningTest = false;
  testRunning = false;

  showBlocklyTerminal = false;
  outputBlockly = '';
  isRunningBlockly = false;

  showSwarmTerminal = false;
  isRunningSwarm = false;
  outputSwarm = '';


  outputLaunch = '';
  outputCmdVel = '';
  outputTeleop = '';



  constructor(private http: HttpClient) { }


  toggleSwarmTerminal() {
    this.showTestFlight = false;
    this.showBlocklyTerminal = false;
    this.showSwarmTerminal = !this.showSwarmTerminal;
  }

  runSwarm() {
    this.outputSwarm = '$ python3 swarm2Tello.py\n';
    this.isRunningSwarm = true;

    this.http.post<{ output: string }>('http://localhost:12345/ros/run-swarm', {}).subscribe({
      next: (res) => {
        this.outputSwarm += res.output;
        this.isRunningSwarm = false;
      },
      error: (err) => {
        this.outputSwarm += `\nErro: ${err.error?.error || err.message}`;
        this.isRunningSwarm = false;
      }
    });
  }

  toggleBlocklyTerminal() {
    this.showSwarmTerminal = false;
    this.showTestFlight = false;
    this.showBlocklyTerminal = !this.showBlocklyTerminal;
  }

  runBlocklyCode() {
    this.outputBlockly = '$ python3 codigo_temp.py\n';
    this.isRunningBlockly = true;

    this.http.post<{ output: string }>('http://localhost:12345/ros/run-blockly', {}).subscribe({
      next: (res) => {
        this.outputBlockly += res.output;
        this.isRunningBlockly = false;
      },
      error: (err) => {
        this.outputBlockly += `\nErro: ${err.error?.error || err.message}`;
        this.isRunningBlockly = false;
      }
    });
  }


  activateTestFlight() {
    this.showSwarmTerminal = false;
    this.showBlocklyTerminal = false;
    this.showTestFlight = !this.showTestFlight;
  }

  runTestFlight() {
    if (this.testRunning) {
      this.killTestFlight();
      return;
    }

    this.isRunningTest = true;
    this.outputLaunch = '';
    this.outputCmdVel = '';
    this.outputTeleop = '';
    this.testRunning = true;

    // 1. Launch tello driver
    this.outputLaunch = '$ ros2 launch tello_driver teleop_launch.py\n';
    this.http.post<{ output: string }>('http://localhost:12345/ros/launch-tello-driver', {}).subscribe({
      next: (res) => this.outputLaunch += res.output + '\n',
      error: (err) => this.outputLaunch += `\nErro: ${err.error?.error || err.message}`
    });

    setTimeout(() => {
      // 2. Echo /cmd_vel (long running)
      this.outputCmdVel = '$ ros2 topic echo /cmd_vel\n';
      this.http.post<{ output: string }>('http://localhost:12345/ros/echo-cmdvel', {}).subscribe({
        next: (res) => this.outputCmdVel += res.output + '\n',
        error: (err) => this.outputCmdVel += `\nErro: ${err.error?.error || err.message}`
      });

      // 3. Teleop
      this.outputTeleop = '$ ros2 run teleop_twist_keyboard teleop_twist_keyboard\n';
      this.http.post<{ output: string }>('http://localhost:12345/ros/tello-keyboard', {}).subscribe({
        next: (res) => this.outputTeleop += res.output + '\n',
        error: (err) => this.outputTeleop += `\nErro: ${err.error?.error || err.message}`
      });
    }, 5000);

    this.isRunningTest = false;
  }

  killTestFlight() {
    this.http.post<{ output: any }>('http://localhost:12345/ros/kill-test-flight', {}).subscribe({
      next: (res) => {
        // var cmd = res && res.output && res.output.cmd_vel ? 'Succeed' : 'Failed' + '\n';;
        // var driver = res && res.output && res.output.driver ? 'Succeed' : 'Failed' + '\n';;
        // var teleop = res && res.output && res.output.teleop ? 'Succeed' : 'Failed' + '\n';;

        this.outputLaunch += '\n[KILL]\n';
        this.outputCmdVel += '\n[KILL]\n';
        this.outputTeleop += '\n[KILL]\n';
      },
      error: (err) => {
        const errorMsg = `\n[KILL ERRO] ${err.error?.error || err.message}`;
        this.outputLaunch += errorMsg;
        this.outputCmdVel += errorMsg;
        this.outputTeleop += errorMsg;
      }
    });
    this.testRunning = false;
  }
}