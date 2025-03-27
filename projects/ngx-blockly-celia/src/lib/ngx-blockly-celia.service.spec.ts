import { TestBed } from '@angular/core/testing';

import { NgxBlocklyCeliaService } from './ngx-blockly-celia.service';

describe('NgxBlocklyCeliaService', () => {
  let service: NgxBlocklyCeliaService;

  beforeEach(() => {
    TestBed.configureTestingModule({});
    service = TestBed.inject(NgxBlocklyCeliaService);
  });

  it('should be created', () => {
    expect(service).toBeTruthy();
  });
});
