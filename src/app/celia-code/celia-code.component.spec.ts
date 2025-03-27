import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CeliaCodeComponent } from './celia-code.component';

describe('CeliaCodeComponent', () => {
  let component: CeliaCodeComponent;
  let fixture: ComponentFixture<CeliaCodeComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [CeliaCodeComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CeliaCodeComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
