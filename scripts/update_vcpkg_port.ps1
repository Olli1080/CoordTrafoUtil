param (
    [Parameter(Mandatory=$true)]
    [string]$Version
)

$repo = "Olli1080/CoordTrafoUtil"
$url = "https://github.com/$repo/archive/refs/tags/v$Version.tar.gz"
$tempFile = "temp_archive.tar.gz"

Write-Host "Downloading archive from $url..." -ForegroundColor Cyan
try {
    Invoke-WebRequest -Uri $url -OutFile $tempFile
} catch {
    Write-Error "Failed to download archive. Ensure the version v$Version exists on GitHub."
    exit 1
}

Write-Host "Calculating SHA512 hash..." -ForegroundColor Cyan
$hash = (Get-FileHash -Path $tempFile -Algorithm SHA512).Hash.ToLower()
Remove-Item $tempFile

$portfilePath = "vcpkg-port/portfile.cmake"
$vcpkgJsonPath = "vcpkg-port/vcpkg.json"

Write-Host "Updating $portfilePath..." -ForegroundColor Cyan
$portfileContent = Get-Content $portfilePath
$portfileContent = $portfileContent -replace 'REF "v\$\{VERSION\}"', "REF v$Version"
$portfileContent = $portfileContent -replace 'SHA512 [a-f0-9]+', "SHA512 $hash"
$portfileContent = $portfileContent -replace 'SHA512 # TODO:.*', "SHA512 $hash"
Set-Content -Path $portfilePath -Value $portfileContent

Write-Host "Updating version in $vcpkgJsonPath..." -ForegroundColor Cyan
$jsonContent = Get-Content $vcpkgJsonPath -Raw | ConvertFrom-Json
$jsonContent.version = $Version
$jsonContent | ConvertTo-Json -Depth 10 | Set-Content $vcpkgJsonPath

Write-Host "Successfully updated vcpkg port to version $Version!" -ForegroundColor Green
Write-Host "Hash: $hash"
