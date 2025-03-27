import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CeliaRunComponent } from './celia-run.component';

describe('CeliaRunComponent', () => {
  let component: CeliaRunComponent;
  let fixture: ComponentFixture<CeliaRunComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [CeliaRunComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CeliaRunComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
