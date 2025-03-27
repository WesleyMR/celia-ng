import { ComponentFixture, TestBed } from '@angular/core/testing';

import { NgxBlocklyCeliaComponent } from './ngx-blockly-celia.component';

describe('NgxBlocklyCeliaComponent', () => {
  let component: NgxBlocklyCeliaComponent;
  let fixture: ComponentFixture<NgxBlocklyCeliaComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      imports: [NgxBlocklyCeliaComponent]
    })
    .compileComponents();

    fixture = TestBed.createComponent(NgxBlocklyCeliaComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
