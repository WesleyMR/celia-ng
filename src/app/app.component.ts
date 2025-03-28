import { Component } from '@angular/core';
import { BlocklyEditorComponent } from './blockly-editor/blockly-editor.component';
import { RouterOutlet, RouterLink, RouterLinkActive } from '@angular/router'; // ✅
import { CommonModule } from '@angular/common';
import { TopbarComponent } from './shared/topbar/topbar.component';
import { MatDialogModule } from '@angular/material/dialog';
import { FormsModule } from '@angular/forms';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';


@Component({
  selector: 'app-root',
  standalone: true,
  imports: [
    CommonModule,
    RouterOutlet,
    RouterLink,
    RouterLinkActive,
    TopbarComponent,
    BlocklyEditorComponent,
    MatDialogModule,
    FormsModule,
    MatFormFieldModule,
    MatInputModule
  ],
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.less']
})

export class AppComponent {
  title = 'celia-ng';
  mostrarSidebar = true;

  blocks = [
    {
      option: 'default',
      type: 'unremovable_block',
      label: 'Tello Python Imports',
      python: () => 'from djitellopy import Tello\n' + 'import cv2\n' + 'import cv2, math, time\n\n' + 'tello = Tello()\n' + 'tello.connect()\n\n' + 'tello.streamon()\n' + 'frame_read = tello.get_frame_read()\n\n',
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "This block cannot be removed.",
      deletable: false
    },
    {
      option: 'default',
      type: 'takeoff',
      label: 'Takeoff',
      python: () => 'tello.takeoff()\n',
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Cause the drone to leave the ground."
    },
    {
      option: 'default',
      type: 'land',
      label: 'Land Drone',
      python: () => 'tello.land()\n',
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Cause the drone to return to the ground."
    },
    {
      option: 'default',
      type: 'take_picture',
      label: 'Take Picture',
      python: () => 'cv2.imwrite("picture.png", frame_read.frame)\n',
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Take Picture"
    },
    {
      option: 'value',
      type: 'front',
      label: 'Forward at',
      secondaryLabel: 'speed',
      python: (speed: number) => `tello.move_forward(${speed})\n`,
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Move the drone forward at 0-100% speed."
    },
    {
      option: 'value',
      type: 'back',
      label: 'Backward at',
      secondaryLabel: 'speed',
      python: (speed: number) => `tello.move_back(${speed})\n`,
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Move the drone back at 0-100% speed."
    },
    {
      option: 'value',
      type: 'right',
      label: 'Right at',
      secondaryLabel: 'speed',
      python: (speed: number) => `tello.move_right(${speed})\n`,
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Move the drone right at 0-100% speed."
    },
    {
      option: 'value',
      type: 'left',
      label: 'Left at',
      secondaryLabel: 'speed',
      python: (speed: number) => `tello.move_left(${speed})\n`,
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Move the drone left at 0-100% speed."
    },
    {
      option: 'value',
      type: 'wait',
      label: 'Wait',
      secondaryLabel: 'duration',
      python: (duration: number) => `time.sleep('${duration})\n`,
      url: 'https://github.com/WesleyMR/blockly-celia/wiki/teste',
      tooltip: "Wait N seconds before calling the next statement."
    }
    
  ];
}
