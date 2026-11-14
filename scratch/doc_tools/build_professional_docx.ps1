param(
    [string]$MdPath = "C:\Users\IIT KGP\MATLAB Drive\shovel-dynamics\study_notebook\Volume_01_Chapter_01.md",
    [string]$DocxPath = "C:\Users\IIT KGP\MATLAB Drive\shovel-dynamics\study_notebook\Volume_01_Chapter_01.docx"
)

$ErrorActionPreference = "Stop"

Write-Host "Reading Markdown file: $MdPath"
$lines = [System.IO.File]::ReadAllLines($MdPath, [System.Text.Encoding]::UTF8)

Write-Host "Initializing Word COM object..."
$word = New-Object -ComObject Word.Application
$word.Visible = $false
$word.DisplayAlerts = 0

try {
    $doc = $word.Documents.Add()
    
    # Page setup
    $doc.PageSetup.TopMargin = 72    # 1 inch
    $doc.PageSetup.BottomMargin = 72
    $doc.PageSetup.LeftMargin = 72
    $doc.PageSetup.RightMargin = 72
    
    # Normal style
    $normalStyle = $doc.Styles.Item("Normal")
    $normalStyle.Font.Name = "Calibri"
    $normalStyle.Font.Size = 11
    $normalStyle.Font.ColorIndex = 1 # Black
    $normalStyle.ParagraphFormat.LineSpacingRule = 0 # Single
    $normalStyle.ParagraphFormat.LineSpacing = 14
    $normalStyle.ParagraphFormat.SpaceAfter = 6
    $normalStyle.ParagraphFormat.SpaceBefore = 0

    $inCodeBlock = $false
    $codeBuffer = New-Object System.Collections.Generic.List[string]
    $inTable = $false
    $tableRows = New-Object System.Collections.Generic.List[string]
    $inDisplayMath = $false
    $mathBuffer = New-Object System.Collections.Generic.List[string]

    function Add-Paragraph([string]$text, [string]$style = "Normal", [int]$spaceBefore = 0, [int]$spaceAfter = 6, [bool]$pageBreak = $false) {
        $p = $doc.Paragraphs.Add()
        $r = $p.Range
        if ($pageBreak) {
            $r.InsertBreak(7) # wdPageBreak
            $p = $doc.Paragraphs.Add()
            $r = $p.Range
        }
        $r.Text = $text
        if ($style -ne "Normal") {
            try { $r.Style = $style } catch {}
        }
        $r.ParagraphFormat.SpaceBefore = $spaceBefore
        $r.ParagraphFormat.SpaceAfter = $spaceAfter
        return $p
    }

    function Flush-CodeBlock() {
        if ($codeBuffer.Count -gt 0) {
            $codeText = [string]::Join("`r`n", $codeBuffer)
            $p = $doc.Paragraphs.Add()
            $table = $doc.Tables.Add($p.Range, 1, 1)
            $table.Borders.Enable = 1
            $table.Borders.OutsideColor = 13750734 # light gray
            $table.Rows.Item(1).Cells.Item(1).Shading.BackgroundPatternColor = 16250868 # #F4F6F8
            $cellRange = $table.Rows.Item(1).Cells.Item(1).Range
            $cellRange.Font.Name = "Consolas"
            $cellRange.Font.Size = 9
            $cellRange.Font.ColorIndex = 1
            $cellRange.Text = $codeText
            $cellRange.ParagraphFormat.SpaceBefore = 0
            $cellRange.ParagraphFormat.SpaceAfter = 0
            $cellRange.ParagraphFormat.LineSpacing = 12
            $codeBuffer.Clear()
            # Move cursor past table
            $doc.Paragraphs.Add()
        }
    }

    function Flush-Table() {
        if ($tableRows.Count -gt 0) {
            $numRows = $tableRows.Count
            $firstRowCols = ($tableRows[0].Trim('|').Split('|')).Count
            $p = $doc.Paragraphs.Add()
            $table = $doc.Tables.Add($p.Range, $numRows, $firstRowCols)
            $table.Borders.Enable = 1
            $table.Borders.OutsideColor = 13750734
            $table.Borders.InsideColor = 14474460

            for ($rIdx = 0; $rIdx -lt $numRows; $rIdx++) {
                $cols = $tableRows[$rIdx].Trim('|').Split('|')
                $rowObj = $table.Rows.Item($rIdx + 1)
                if ($rIdx -eq 0) {
                    $rowObj.Shading.BackgroundPatternColor = 15921906 # #F1F5F9
                    $rowObj.HeadingFormat = -1 # Repeat header
                }
                for ($cIdx = 0; $cIdx -lt [Math]::Min($cols.Count, $firstRowCols); $cIdx++) {
                    $cellRange = $rowObj.Cells.Item($cIdx + 1).Range
                    $cellRange.Font.Name = "Calibri"
                    $cellRange.Font.Size = 10
                    if ($rIdx -eq 0) {
                        $cellRange.Font.Bold = 1
                    }
                    $cellRange.Text = $cols[$cIdx].Trim()
                    $cellRange.ParagraphFormat.SpaceBefore = 2
                    $cellRange.ParagraphFormat.SpaceAfter = 2
                }
            }
            $tableRows.Clear()
            $doc.Paragraphs.Add()
        }
    }

    function Flush-DisplayMath() {
        if ($mathBuffer.Count -gt 0) {
            $mathText = [string]::Join(" ", $mathBuffer)
            $p = $doc.Paragraphs.Add()
            $r = $p.Range
            $r.Text = $mathText
            $r.ParagraphFormat.Alignment = 1 # Center
            $r.ParagraphFormat.SpaceBefore = 6
            $r.ParagraphFormat.SpaceAfter = 6
            try {
                if ($p.Range.End -gt $p.Range.Start + 1) {
                    $rClean = $doc.Range($p.Range.Start, $p.Range.End - 1)
                    [void]$doc.OMaths.Add($rClean)
                }
            } catch {
                Write-Warning "Failed to add display math: $_"
            }
            $mathBuffer.Clear()
        }
    }

    Write-Host "Parsing Markdown and generating Word elements..."
    $i = 0
    $h1Count = 0

    while ($i -lt $lines.Length) {
        $line = $lines[$i]

        # Code block
        if ($line.StartsWith('```')) {
            if ($inCodeBlock) {
                $inCodeBlock = $false
                Flush-CodeBlock
            } else {
                Flush-Table
                Flush-DisplayMath
                $inCodeBlock = $true
            }
            $i++
            continue
        }

        if ($inCodeBlock) {
            $codeBuffer.Add($line)
            $i++
            continue
        }

        # Display math $$
        if ($line.Trim().StartsWith('$$')) {
            Flush-Table
            if ($line.Trim().EndsWith('$$') -and $line.Trim().Length -gt 2) {
                $eq = $line.Trim().Substring(2, $line.Trim().Length - 4).Trim()
                $mathBuffer.Add($eq)
                Flush-DisplayMath
            } else {
                $inDisplayMath = $true
                $firstPart = $line.Trim().Substring(2).Trim()
                if ($firstPart) { $mathBuffer.Add($firstPart) }
            }
            $i++
            continue
        }

        if ($inDisplayMath) {
            if ($line.Trim().EndsWith('$$')) {
                $inDisplayMath = $false
                $lastPart = $line.Trim().Substring(0, $line.Trim().Length - 2).Trim()
                if ($lastPart) { $mathBuffer.Add($lastPart) }
                Flush-DisplayMath
            } else {
                $mathBuffer.Add($line.Trim())
            }
            $i++
            continue
        }

        # Empty line
        if ([string]::IsNullOrWhiteSpace($line)) {
            Flush-Table
            $i++
            continue
        }

        # Horizontal rule
        if ($line.Trim() -in @('---', '***', '___')) {
            Flush-Table
            $i++
            continue
        }

        # Markdown tables
        if ($line.Contains('|') -and $line.Trim().StartsWith('|') -and $line.Trim().EndsWith('|')) {
            if ($line -match '^\s*\|(?:\s*:?-+:?\s*\|)+\s*$') {
                # Delimiter row
                $i++
                continue
            }
            $tableRows.Add($line)
            $i++
            continue
        } else {
            Flush-Table
        }

        # Headings
        if ($line.StartsWith('# ')) {
            $h1Count++
            $pageBreak = ($h1Count -gt 1)
            $p = Add-Paragraph -text $line.Substring(2).Trim() -style "Heading 1" -spaceBefore 18 -spaceAfter 8 -pageBreak $pageBreak
            $p.Range.Font.Size = 18
            $p.Range.Font.Bold = 1
            $p.Range.Font.ColorIndex = 9 # Dark Blue
            $i++
            continue
        }

        if ($line.StartsWith('## ')) {
            $p = Add-Paragraph -text $line.Substring(3).Trim() -style "Heading 2" -spaceBefore 14 -spaceAfter 6
            $p.Range.Font.Size = 14
            $p.Range.Font.Bold = 1
            $p.Range.Font.ColorIndex = 9
            $i++
            continue
        }

        if ($line.StartsWith('### ')) {
            $p = Add-Paragraph -text $line.Substring(4).Trim() -style "Heading 3" -spaceBefore 10 -spaceAfter 4
            $p.Range.Font.Size = 12
            $p.Range.Font.Bold = 1
            $p.Range.Font.ColorIndex = 9
            $i++
            continue
        }

        if ($line.StartsWith('#### ')) {
            $p = Add-Paragraph -text $line.Substring(5).Trim() -style "Normal" -spaceBefore 8 -spaceAfter 4
            $p.Range.Font.Size = 11
            $p.Range.Font.Bold = 1
            $i++
            continue
        }

        # Blockquote
        if ($line.StartsWith('>')) {
            $qText = $line.Substring(1).Trim()
            $p = $doc.Paragraphs.Add()
            $table = $doc.Tables.Add($p.Range, 1, 1)
            $table.Borders.Enable = 0
            $table.Borders.Item(-4).LineStyle = 1 # Left border wdBorderLeft
            $table.Borders.Item(-4).LineWidth = 24 # 3pt
            $table.Borders.Item(-4).Color = 15372579 # blue
            $cell = $table.Rows.Item(1).Cells.Item(1)
            $cell.Shading.BackgroundPatternColor = 16773867 # #EFF6FF
            $cell.Range.Font.Name = "Calibri"
            $cell.Range.Font.Size = 10.5
            $cell.Range.Font.Italic = 1
            $cell.Range.Text = $qText
            $cell.Range.ParagraphFormat.SpaceBefore = 4
            $cell.Range.ParagraphFormat.SpaceAfter = 4
            $doc.Paragraphs.Add()
            $i++
            continue
        }

        # Lists
        if ($line -match '^\s*[-*]\s+(.*)$') {
            $p = Add-Paragraph -text $matches[1] -style "List Bullet" -spaceBefore 2 -spaceAfter 2
            $i++
            continue
        }

        if ($line -match '^\s*(\d+)\.\s+(.*)$') {
            $p = Add-Paragraph -text "$($matches[1]). $($matches[2])" -style "Normal" -spaceBefore 2 -spaceAfter 2
            $i++
            continue
        }

        # Regular Paragraph
        $p = Add-Paragraph -text $line -style "Normal" -spaceBefore 0 -spaceAfter 6
        $i++
    }

    Flush-CodeBlock
    Flush-Table
    Flush-DisplayMath

    Write-Host "Building up mathematical equations..."
    try {
        $doc.OMaths.BuildUp()
        Write-Host "Equations successfully built up."
    } catch {
        Write-Warning "OMaths BuildUp notice: $_"
    }

    Write-Host "Saving document to: $DocxPath"
    $doc.SaveAs2($DocxPath, 16) # wdFormatXMLDocument = 16
    $doc.Close()
    Write-Host "Word document created successfully: $DocxPath"

} finally {
    $word.Quit()
}
