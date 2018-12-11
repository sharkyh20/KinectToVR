$JSON = Get-Content "$env:LOCALAPPDATA\openvr\openvrpaths.vrpath" | Out-String | ConvertFrom-Json

if ($JSON)
{
  foreach( $steamPath in $JSON.runtime )
  {
    $vrpathreg_path= "$steamPath/bin/win64/vrpathreg.exe"
    if ([System.IO.File]::Exists($vrpathreg_path))
    {
      return $steamPath
    }
  }
}

exit -1