// src/app/shared/services/auth.service.ts
import { Injectable } from '@angular/core';

@Injectable({ providedIn: 'root' })
export class AuthService {
    private currentUser: string | null = null;

    setUser(username: string) {
        this.currentUser = username;
    }

    getUser(): string | null {
        return this.currentUser;
    }

    isLoggedIn(): boolean {
        return !!this.currentUser;
    }

    logout() {
        this.currentUser = null;
    }
}
