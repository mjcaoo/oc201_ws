# Convert .c and .h files to UTF-8 encoding (no BOM)
# Handles files encoded with GBK/GB2312 or UTF-8 with BOM

$files = Get-ChildItem -Path . -Recurse -Include *.c, *.h
$converted = 0
$skipped = 0
$errors = 0

foreach ($file in $files) {
    try {
        $bytes = [System.IO.File]::ReadAllBytes($file.FullName)
        
        # Check for UTF-8 BOM (EF BB BF)
        $hasBOM = $bytes.Length -ge 3 -and $bytes[0] -eq 0xEF -and $bytes[1] -eq 0xBB -and $bytes[2] -eq 0xBF
        
        # Remove BOM if present
        if ($hasBOM) {
            $bytes = $bytes[3..($bytes.Length-1)]
            Write-Host "Removed BOM from: $($file.FullName)"
        }
        
        # Try UTF-8 decoding
        $utf8String = [System.Text.Encoding]::UTF8.GetString($bytes)
        $hasReplacementChar = $utf8String.Contains([char]0xFFFD)
        
        # Check if UTF-8 decoding is valid and contains non-ASCII characters
        $nonAsciiPattern = '[^\x00-\x7F]'
        $hasNonAscii = $utf8String -match $nonAsciiPattern
        
        # If UTF-8 decoding is valid (no replacement chars) and has non-ASCII, it's likely UTF-8
        if (-not $hasReplacementChar -and $hasNonAscii) {
            # Already UTF-8, but if we removed BOM, need to save again
            if ($hasBOM) {
                $utf8Bytes = [System.Text.Encoding]::UTF8.GetBytes($utf8String)
                [System.IO.File]::WriteAllBytes($file.FullName, $utf8Bytes)
                Write-Host "Saved without BOM: $($file.FullName)"
            }
            $skipped++
            continue
        }
        
        # If no non-ASCII characters, it's pure ASCII (subset of UTF-8)
        if (-not $hasNonAscii) {
            if ($hasBOM) {
                $utf8Bytes = [System.Text.Encoding]::UTF8.GetBytes($utf8String)
                [System.IO.File]::WriteAllBytes($file.FullName, $utf8Bytes)
                Write-Host "Saved ASCII without BOM: $($file.FullName)"
            }
            $skipped++
            continue
        }
        
        # Try GBK decoding
        $gbkEncoding = [System.Text.Encoding]::GetEncoding("gb2312")
        $gbkString = $gbkEncoding.GetString($bytes)
        $gbkHasReplacementChar = $gbkString.Contains([char]0xFFFD)
        
        # If GBK decoding is valid (no replacement chars) and contains non-ASCII
        if (-not $gbkHasReplacementChar) {
            # Convert to UTF-8
            $utf8Bytes = [System.Text.Encoding]::UTF8.GetBytes($gbkString)
            [System.IO.File]::WriteAllBytes($file.FullName, $utf8Bytes)
            Write-Host "Converted from GBK to UTF-8: $($file.FullName)"
            $converted++
        } else {
            Write-Host "Skipping (unknown encoding): $($file.FullName)"
            $errors++
        }
    } catch {
        Write-Host "Error processing $($file.FullName): $_"
        $errors++
    }
}

Write-Host "`nSummary:"
Write-Host "Converted: $converted files"
Write-Host "Skipped (already UTF-8 or ASCII): $skipped files"
Write-Host "Errors: $errors files"