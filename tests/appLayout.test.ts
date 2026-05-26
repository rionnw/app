import { describe, expect, test } from "bun:test";

const appSource = await Bun.file(new URL("../src/App.tsx", import.meta.url)).text();
const appStyles = await Bun.file(new URL("../src/App.css", import.meta.url)).text();

describe("app image layout", () => {
  test("keeps ROI annotation available above the image and out of the bottom action bar", () => {
    const imagePanel = appSource.slice(
      appSource.indexOf('<section className="image-panel">'),
      appSource.indexOf('<aside className="side-panel">'),
    );
    const beforeImageStage = imagePanel.slice(0, imagePanel.indexOf('className={`image-stage'));
    const bottomActions = imagePanel.slice(
      imagePanel.indexOf('<div className="bottom-actions">'),
      imagePanel.indexOf("</div>", imagePanel.indexOf('<div className="bottom-actions">')),
    );

    expect(beforeImageStage).toContain("标注 ROI");
    expect(bottomActions).not.toContain("标注 ROI");
    expect(bottomActions.match(/<button/g)).toHaveLength(3);
  });

  test("lets the image panel header wrap before the ROI button is clipped", () => {
    expect(appStyles).toMatch(/\.panel-title\s*\{[^}]*flex-wrap:\s*wrap;/);
    expect(appStyles).toMatch(/\.panel-heading\s*\{[^}]*flex:\s*0 0 auto;/);
    expect(appStyles).toMatch(/\.panel-heading button\s*\{[^}]*white-space:\s*nowrap;/);
  });
});
