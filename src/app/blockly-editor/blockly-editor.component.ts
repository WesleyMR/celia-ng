import { Component, AfterViewInit, ViewChild, ElementRef, Input } from '@angular/core';
import * as Blockly from 'blockly';
import { pythonGenerator } from 'blockly/python';
import { FormsModule } from '@angular/forms'; //
import { CommonModule } from '@angular/common';
import { ChangeDetectorRef } from '@angular/core';

export interface CustomBlock {
  option: string;
  type: string;
  label: string;
  secondaryLabel?: string;
  color?: number;
  python: ((value?: any) => string);
  tooltip?: string;
  url?: string;
  deletable?: boolean;
}

@Component({
  selector: 'app-blockly-editor',
  standalone: true,
  templateUrl: './blockly-editor.component.html',
  styleUrls: ['./blockly-editor.component.less'],
  imports: [CommonModule, FormsModule]
})
export class BlocklyEditorComponent implements AfterViewInit {
  @ViewChild('blocklyDiv') blocklyDiv!: ElementRef;
  @ViewChild('editorDiv') editorDiv!: ElementRef<HTMLDivElement>;

  @Input() blocks: CustomBlock[] = [];

  workspace: Blockly.WorkspaceSvg | undefined;
  codigoPython: string = '';
  modoCodigo = false;
  mostrarConfirmacao = false;
  mostrarBlockly = true;
  mostrarBlocklyEditor = true;
  mensagemModal: string | null = null;
  
  constructor(private cdr: ChangeDetectorRef) {}
  // ngAfterViewInit(): void {
  //   if (this.mostrarBlocklyEditor) {
  //     this.inicializarBlockly();
  //     this.gerarCodigoPython();
  //   }
  // }
  ngAfterViewInit(): void {
    if (this.mostrarBlocklyEditor && !this.modoCodigo) {
      this.inicializarBlockly();
      this.gerarCodigoPython();
    }
  
    // Se for modo edição e já tiver código, injeta
    if (this.modoCodigo && this.codigoPython) {
      this.atualizarEditorTextoManual();
    }
  }
  
  atualizarEditorTextoManual(): void {
    if (this.editorDiv) {
      this.editorDiv.nativeElement.innerText = this.codigoPython;
    }
  }

  inicializarBlockly(): void {
    if (!this.blocklyDiv?.nativeElement) return;

    this.defineCustomBlocks();

    const toolbox = {
      kind: 'flyoutToolbox',
      contents: this.blocks
        .filter(block => block.type !== 'unremovable_block')
        .map(block => ({ kind: 'block', type: block.type }))
    };

    this.workspace = Blockly.inject(this.blocklyDiv.nativeElement, { toolbox });

    const importBlock = this.workspace.newBlock('unremovable_block');
    importBlock.initSvg();
    importBlock.render();
    importBlock.setDeletable(false);
    importBlock.setMovable(false);
    importBlock.setEditable(false);
    importBlock.moveBy(20, 20);

    // Conectar blocos automaticamente
    // this.workspace.addChangeListener((event) => {
    //   if (event.type !== Blockly.Events.BLOCK_CREATE) return;

    //   const blockCreateEvent = event as Blockly.Events.BlockCreate;
    //   const blockIds = blockCreateEvent.ids ?? [];

    //   requestAnimationFrame(() => {
    //     const createdBlocks: Blockly.BlockSvg[] = blockIds
    //     .map(id => this.workspace!.getBlockById(id))
    //     .filter((b): b is Blockly.BlockSvg => !!b && b.type !== 'unremovable_block');


    //     if (createdBlocks.length > 1) {
    //       for (let i = 0; i < createdBlocks.length - 1; i++) {
    //         const current = createdBlocks[i];
    //         const next = createdBlocks[i + 1];

    //         if (current.nextConnection && next.previousConnection) {
    //           current.nextConnection.connect(next.previousConnection);
    //         }
    //       }
    //     } else if (createdBlocks.length === 1) {
    //       const newBlock = createdBlocks[0];
    //       const topBlocks = this.workspace!.getTopBlocks(true);
    //       const previousBlock = topBlocks.find(b => b.id !== newBlock.id && b.nextConnection);

    //       if (previousBlock?.nextConnection && newBlock.previousConnection) {
    //         try {
    //           previousBlock.nextConnection.connect(newBlock.previousConnection);
    //         } catch (err) {
    //           console.warn("Erro ao conectar blocos automaticamente:", err);
    //         }
    //       }
    //     }
    //   });
    // });
  }

  defineCustomBlocks(): void {
    for (const block of this.blocks) {
      const color = this.color();

      switch (block.option) {
        case 'default':
          Blockly.Blocks[block.type] = {
            init: function () {
              this.appendDummyInput().appendField(block.label);
              this.setColour(color);
              this.setPreviousStatement(true, null);
              this.setNextStatement(true, null);
              this.setTooltip(block.tooltip);
              this.setDeletable(block.deletable ?? true);
              this.setHelpUrl(block.url);
            }
          };
          pythonGenerator.forBlock[block.type] = block.python;
          break;

        case 'value':
          Blockly.Blocks[block.type] = {
            init: function () {
              this.appendDummyInput()
                .appendField(block.label)
                .appendField(new Blockly.FieldNumber(0, 0, 100), block.secondaryLabel)
                .appendField(block.secondaryLabel);

              this.setColour(color);
              this.setPreviousStatement(true, null);
              this.setNextStatement(true, null);
              this.setTooltip(block.tooltip);
              this.setDeletable(block.deletable ?? true);
              this.setHelpUrl(block.url);
            }
          };

          pythonGenerator.forBlock[block.type] = function (blockInstance, generator) {
            const value = blockInstance.getFieldValue(block.secondaryLabel as string);
            const code = typeof block.python === 'function'
              ? block.python.length === 1 ? block.python(value) : block.python()
              : block.python;
            return code;
          };
          break;
      }
    }
  }

  color(): number {
    return Math.floor(Math.random() * 360 + 1);
  }

  gerarCodigoPython(): void {
    if (this.workspace) {
      this.codigoPython = pythonGenerator.workspaceToCode(this.workspace);
      this.cdr.detectChanges(); // <- aqui está o segredo!
    }
  
  }

  
  // gerarCodigoPython(): void {
  //   if (!this.workspace) return;

  //   const allBlocks = this.workspace.getTopBlocks(true);
  //   const fixedBlock = allBlocks.find(b => b.type === 'unremovable_block');
  //   const outrosBlocos = allBlocks.filter(b => b.type !== 'unremovable_block');

  //   let codigo = '';

  //   // Gera o código do bloco fixo primeiro
  //   if (fixedBlock) {
  //     codigo += pythonGenerator.blockToCode(fixedBlock);
  //     codigo += '\n';
  //   }

  //   // Gera o código dos demais blocos
  //   for (const block of outrosBlocos) {
  //     codigo += pythonGenerator.blockToCode(block);
  //   }

  //   this.codigoPython = codigo;
  // }

  baixarArquivoPython(): void {
    const blob = new Blob([this.codigoPython], { type: 'text/x-python' });
    const url = window.URL.createObjectURL(blob);
    const link = document.createElement('a');
    link.href = url;
    link.download = 'codigo.py';
    link.click();
    window.URL.revokeObjectURL(url);
  }

  toggleModoCodigo(): void {
    if (this.modoCodigo) {
      this.mostrarConfirmacao = true;
    } else {
      this.modoCodigo = true;
      (this.workspace as any)?.setVisible(false);
      this.gerarCodigoPython();  // Garante que está atualizado
      setTimeout(() => this.atualizarEditorTextoManual(), 0);
    }
  }
  // confirmarTroca(): void {
  //   this.modoCodigo = false;
  //   this.mostrarConfirmacao = false;

  //   this.codigoPython = '';

  //   this.mostrarBlockly = false; // força destruição do div
  //   setTimeout(() => {
  //     this.mostrarBlockly = true; // força recriação da div
  //   }, 0);
  // }

  // confirmarTroca(): void {
  //   window.location.reload();
  // }
  confirmarTroca(): void {
    this.modoCodigo = false;
    this.mostrarConfirmacao = false;
    this.codigoPython = '';

    // Força a destruição do Blockly
    this.mostrarBlocklyEditor = false;

    setTimeout(() => {
      this.mostrarBlocklyEditor = true;
      setTimeout(() => {
        this.inicializarBlockly();
        this.gerarCodigoPython();
      }, 0);
    }, 0);
  }
  cancelarTroca(): void {
    this.mostrarConfirmacao = false;
  }

  abrirNoVSCode(): void {
    fetch('http://localhost:12345/abrir-vscode', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify({ codigo: this.codigoPython })
    }).then(response => {
      if (response.ok) {
        this.mensagemModal = 'Código aberto no VS Code!';
      } else {
        this.mensagemModal = 'Erro ao abrir o VS Code.';
      }
    }).catch(error => {
      console.error('Erro na requisição:', error);
      this.mensagemModal = 'Erro ao conectar com o backend.';
    });
  } 
  onCodigoEditado(event: Event): void {
    const div = event.target as HTMLDivElement;
    this.codigoPython = div.innerText;
  }
   
}
