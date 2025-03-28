# ngx-blockly-celia

A customizable Blockly editor component for Angular that generates Python code from visual blocks and supports export to `.py` files.

![npm](https://img.shields.io/npm/v/ngx-blockly-celia)
![Angular](https://img.shields.io/badge/angular-standalone%20component-green)
![License](https://img.shields.io/npm/l/ngx-blockly-celia)

---

## ✨ Features

- Define custom blocks using `@Input()`
- Python code generation (`print("...")`)
- Export generated code as `.py` with one click
- Compatible with Blockly v11+
- Fully standalone component (Angular 15+)

---

## 🚀 Installation

```bash
npm install ngx-blockly-celia blockly
```

> ⚠️ **Important:** This library declares `blockly` as a `peerDependency`. You must install it alongside.

---

## 🧠 Example usage

```ts
import { BlocklyEditorComponent } from 'ngx-blockly-celia';

@Component({
  standalone: true,
  imports: [BlocklyEditorComponent],
  ...
})
export class AppComponent {
  blocks = [
    {
      type: 'say_hello',
      label: 'Say Hello',
      color: 160,
      python: () => 'print("Hello")\n',
      tooltip: 'Prints Hello',
      url: 'https://example.com'
    }
  ];
}
```

```html
<app-blockly-editor [blocks]="blocks"></app-blockly-editor>
```

---

## 🧱 Block schema

Each block definition can include the following fields:

| Field     | Type             | Required | Description                         |
|-----------|------------------|----------|-------------------------------------|
| `type`    | `string`         | ✅ Yes   | Unique block type name              |
| `label`   | `string`         | ✅ Yes   | Text displayed on the block         |
| `color`   | `number`         | ❌ No    | HSL hue (0–360)                     |
| `tooltip` | `string`         | ❌ No    | Shown on mouse hover                |
| `url`     | `string`         | ❌ No    | Help link (opens on question mark) |
| `python`  | `() => string`   | ✅ Yes   | Function that returns Python code   |

---

## 🔗 Links

- [Blockly documentation](https://developers.google.com/blockly)
- [Angular](https://angular.io/)

---

## 🪪 License

MIT © Wesley