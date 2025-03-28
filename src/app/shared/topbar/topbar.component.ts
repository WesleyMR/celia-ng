import { Component } from '@angular/core';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatIconModule } from '@angular/material/icon';
import { MatButtonModule } from '@angular/material/button';
import { MatDialog } from '@angular/material/dialog';
import { LoginDialogComponent } from '../login-dialog/login-dialog.component';
import { AuthService } from '../services/auth.service';
import { CommonModule } from '@angular/common';
import { MatMenuModule } from '@angular/material/menu';

@Component({
  selector: 'app-topbar',
  imports: [CommonModule, MatButtonModule, MatIconModule, MatToolbarModule, MatMenuModule],
  templateUrl: './topbar.component.html',
  styleUrl: './topbar.component.less'
})
export class TopbarComponent {
  // username: string | null = null;

  // constructor(private dialog: MatDialog, public authService: AuthService) { }

  // openLoginDialog() {
  //   const dialogRef = this.dialog.open(LoginDialogComponent);

  //   dialogRef.afterClosed().subscribe(result => {
  //     if (result) {
  //       console.log('Usuário logado:', result);
  //       this.authService.setUsername(this.username);
  //       this.dialogRef.close(this.username);
  //     }
  //   });
  // }
  constructor(private dialog: MatDialog, public authService: AuthService) {}

  openLoginDialog() {
    const dialogRef = this.dialog.open(LoginDialogComponent);

    dialogRef.afterClosed().subscribe(result => {
      if (result) {
        console.log('Usuário logado:', result);
        this.authService.setUser(result);
      }
    });
  }

  get username(): string | null {
    return this.authService.getUser();
  }

  logout() {
    this.authService.logout();
  }
}