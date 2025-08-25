<h1 align="center">
  2025 Control

  <p align="center">
      <a href="https://github.com/recordrobotics/2025_Control/actions/workflows/ci.yml"><img alt="CI" src="https://github.com/recordrobotics/2025_Control/actions/workflows/ci.yml/badge.svg?branch=main"></a>
      <a href="https://docs.recordrobotics.org/"><img alt="Read the Docs" src="https://img.shields.io/readthedocs/2024-control?logo=readthedocs&labelColor=%23556bc2"></a>
      <a href="https://github.com/recordrobotics/2025_Control/actions/workflows/test.yml"><img alt="Unit Tests Status" src="https://img.shields.io/github/actions/workflow/status/recordrobotics/2025_Control/test.yml?logo=gradle&label=tests&labelColor=purple"></a>
  </p>
</h1>

Here are our [goals](https://recordrobotics.notion.site/1714851f43d58095ac37c44f40ad3b70?v=1714851f43d5800d9462000c01589958) for the season.

Docs at [docs.recordrobotics.org](https://docs.recordrobotics.org/)

---

## Extensions

This repository recommends the following VS Code extensions for the best development experience:

<table>
  <tr>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=streetsidesoftware.code-spell-checker">
        <img src="https://streetsidesoftware.gallerycdn.vsassets.io/extensions/streetsidesoftware/code-spell-checker/4.2.3/1753028947698/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="Code Spell Checker"/><br/>
        <b>Code Spell Checker</b>
      </a>
    </td>
    <td align="center">
      <a href="https://marketplace.visualstudio.com/items?itemName=SonarSource.sonarlint-vscode">
        <img src="https://sonarsource.gallerycdn.vsassets.io/extensions/sonarsource/sonarlint-vscode/4.29.0/1755515927519/Microsoft.VisualStudio.Services.Icons.Default" width="64" alt="SonarLint"/><br/>
        <b>SonarLint</b>
      </a>
    </td>
  </tr>
</table>

### SonarLint Setup

To use the custom SonarLint rules configured for this repository, you need to copy the SonarLint rules from [`.vscode/settings.json`](.vscode/settings.json) into your **user settings**:

1. Open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P` on Mac).
2. Type and select **Preferences: Open User Settings (JSON)**.
3. Copy the `"sonarlint.rules"` section from [`.vscode/settings.json`](.vscode/settings.json) into your user settings file.

This ensures SonarLint uses the same code quality rules as the repository.

-------------------------------------------------
[Record Robotics](https://www.recordrobotics.org/)
