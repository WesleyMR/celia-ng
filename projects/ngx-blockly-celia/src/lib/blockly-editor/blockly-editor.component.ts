import { Component, AfterViewInit, ElementRef, ViewChild } from '@angular/core';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';

@Component({
  selector: 'app-blockly-editor',
  standalone: true,
  imports: [],
  templateUrl: './blockly-editor.component.html',
  styleUrls: ['./blockly-editor.component.less']
})
export class BlocklyEditorComponent implements AfterViewInit {
  @ViewChild('blocklyDiv', { static: true }) blocklyDiv!: ElementRef;

  workspace: Blockly.WorkspaceSvg | undefined;
  codigoPython: string = '';

  ngAfterViewInit(): void {
    this.defineCustomBlocks();

    // Injetar o Blockly no elemento após a criação
    this.workspace = Blockly.inject(this.blocklyDiv.nativeElement, {
      toolbox: {
        kind: 'flyoutToolbox',
        contents: [
          { kind: 'block', type: 'say_hello' },
          { kind: 'block', type: 'say_goodbye' }
        ]
      }
    });
  }

  defineCustomBlocks(): void {
    // Define blocos personalizados
    Blockly.Blocks['say_hello'] = {
      init: function () {
        this.appendDummyInput().appendField('Say Hello');
        this.setColour(160);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
      }
    };

    Blockly.Blocks['say_goodbye'] = {
      init: function () {
        this.appendDummyInput().appendField('Say Goodbye');
        this.setColour(210);
        this.setPreviousStatement(true, null);
        this.setNextStatement(true, null);
      }
    };

    // Gerador Python (workaround funcional sem pacote externo)
    //const pythonGenerator = (Blockly as any).Python;

    if (pythonGenerator) {
      pythonGenerator.forBlock = pythonGenerator.forBlock || {};

      pythonGenerator.forBlock['say_hello'] = function () {
        return 'print("Hello")\n';
      };

      pythonGenerator.forBlock['say_goodbye'] = function () {
        return 'print("Goodbye")\n';
      };
    } else {
      console.error('Blockly.Python não encontrado. Verifique a versão do Blockly.');
    }
  }

  gerarCodigoPython(): void {
    if (this.workspace) {
      //const pythonGenerator = (Blockly as any).Python;
      this.codigoPython = pythonGenerator.workspaceToCode(this.workspace);
    }
  }

  baixarArquivoPython(): void {
    const blob = new Blob([this.codigoPython], { type: 'text/x-python' });
    const url = window.URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'codigo.py';
    link.click();
    window.URL.revokeObjectURL(url); // limpeza
  }
  
}
