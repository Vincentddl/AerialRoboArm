import fs from "node:fs/promises";
import path from "node:path";
import { FileBlob, SpreadsheetFile } from "@oai/artifact-tool";

const inputPath =
  "C:\\Users\\Vincent Lu\\Documents\\BOM_20260605 机械臂开发板-REV1 板厚1.6MM_2026-06-07.xlsx";
const outputDir = path.join(process.cwd(), "outputs", "bom_polarity_highlight");
const outputPath = path.join(outputDir, "BOM_20260605_polarity_highlighted_v2.xlsx");

const targets = new Set([
  "C1",
  "C27",
  "LED1",
  "LED2",
  "LED4",
  "LED5",
  "LED8",
  "Q3",
  "Q4",
  "U1",
  "U11",
  "U3",
  "U8",
  "U9",
  "X3",
]);

await fs.mkdir(outputDir, { recursive: true });

const input = await FileBlob.load(inputPath);
const workbook = await SpreadsheetFile.importXlsx(input);
const sheet = workbook.worksheets.getItemAt(0);

const inspected = await workbook.inspect({
  kind: "table",
  range: `${sheet.name}!A1:J62`,
  include: "values",
  tableMaxRows: 70,
  tableMaxCols: 10,
});

const records = inspected.ndjson
  .trim()
  .split(/\r?\n/)
  .filter(Boolean)
  .map((line) => JSON.parse(line));
const table = records.find((record) => record.values)?.values ?? [];
const header = table[0] ?? [];
const designatorCol = header.findIndex((value) => value === "Designator");

if (designatorCol === -1) {
  throw new Error("Designator column was not found.");
}

const highlighted = [];
for (let i = 1; i < table.length; i += 1) {
  const row = table[i];
  const designator = row[designatorCol];
  if (!designator) continue;

  const designators = String(designator)
    .split(",")
    .map((item) => item.trim())
    .filter(Boolean);

  if (designators.some((item) => targets.has(item))) {
    const excelRow = i + 1;
    sheet.getRange(`A${excelRow}:J${excelRow}`).format = {
      fill: "#FFF2CC",
    };
    highlighted.push({ row: excelRow, designator });
  }
}

const output = await SpreadsheetFile.exportXlsx(workbook);
await output.save(outputPath);

console.log(JSON.stringify({ outputPath, highlighted }, null, 2));
