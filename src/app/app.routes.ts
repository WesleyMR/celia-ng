// import { Routes } from '@angular/router';
// export const routes: Routes = [];

//SE QUISER VOLTAR A COMO ERA, DESCOMENTE AS DUAS LINHAS ACIMA

import { Routes } from '@angular/router';
import { BlocklyEditorComponent } from './blockly-editor/blockly-editor.component';
import { CeliaRunComponent } from './celia-run/celia-run.component';
import { CeliaGazeboComponent } from './celia-gazebo/celia-gazebo.component';
import { CeliaCodeComponent } from './celia-code/celia-code.component';

export const routes: Routes = [
  { path: '', redirectTo: '/code', pathMatch: 'full' },
  { path: 'code', component: CeliaCodeComponent }, // ✅ Agora vai para CodeComponent
  { path: 'run', component: CeliaRunComponent },
  { path: 'gazebo', component: CeliaGazeboComponent },
];