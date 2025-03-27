// import { Component } from '@angular/core';

// @Component({
//   selector: 'app-celia-code',
//   imports: [],
//   templateUrl: './celia-code.component.html',
//   styleUrl: './celia-code.component.less'
// })
// export class CeliaCodeComponent {

// }

import { Component, inject } from '@angular/core';
import { BlocklyEditorComponent, CustomBlock } from '../blockly-editor/blockly-editor.component';
import { AppComponent } from '../app.component'; // importar para acessar os blocos definidos lá
import { CommonModule } from '@angular/common';

@Component({
  selector: 'app-code',
  standalone: true,
  imports: [CommonModule, BlocklyEditorComponent],
  template: `<app-blockly-editor [blocks]="blocks" *ngIf="blocks.length"></app-blockly-editor>`,
})
export class CeliaCodeComponent {
  blocks: CustomBlock[] = [];

  constructor() {
    const app = inject(AppComponent);
    this.blocks = app.blocks;
  }
}
