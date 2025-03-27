import { ComponentFixture, TestBed } from '@angular/core/testing';

import { CeliaGazeboComponent } from './celia-gazebo.component';

describe('CeliaGazeboComponent', () => {
  let component: CeliaGazeboComponent;
  let fixture: ComponentFixture<CeliaGazeboComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [CeliaGazeboComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(CeliaGazeboComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
