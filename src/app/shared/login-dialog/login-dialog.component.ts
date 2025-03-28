import { Component } from '@angular/core';
import { MatDialogRef } from '@angular/material/dialog';
import { HttpClient } from '@angular/common/http';
import { MatDialogModule } from '@angular/material/dialog';
import { MatFormFieldModule } from '@angular/material/form-field';
import { MatInputModule } from '@angular/material/input';
import { MatButtonModule } from '@angular/material/button';
import { CommonModule } from '@angular/common';
import { FormsModule } from '@angular/forms';
import { HttpClientModule } from '@angular/common/http';
import { AuthService } from '../services/auth.service';

@Component({
  selector: 'app-login-dialog',
  standalone: true,
  templateUrl: './login-dialog.component.html',
  imports: [MatButtonModule, MatFormFieldModule, MatInputModule, MatDialogModule, CommonModule, FormsModule, HttpClientModule]
})
export class LoginDialogComponent {
  username: string = '';
  password: string = '';

  constructor(
    private dialogRef: MatDialogRef<LoginDialogComponent>,
    private http: HttpClient,
    private authService: AuthService
  ) {}

  login() {
    this.http.post('http://localhost:12345/login', {
      username: this.username,
      password: this.password
    }).subscribe({
      next: (res) => {
        this.authService.setUser(this.username);
        this.dialogRef.close(this.username);
      },
      error: () => {
        alert('Usuário ou senha incorretos');
      }
    });
  }

  cancel() {
    this.dialogRef.close();
  }
}
