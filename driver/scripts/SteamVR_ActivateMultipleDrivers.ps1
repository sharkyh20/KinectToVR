$OpenVRPathsFile= "$env:LOCALAPPDATA\openvr\openvrpaths.vrpath"
$OpenVRPathsJSON = Get-Content $OpenVRPathsFile | Out-String | ConvertFrom-Json

if ($OpenVRPathsJSON)
{
  foreach( $steamPath in $OpenVRPathsJSON.config )
  {
    $test_vrsettings_path= "$steamPath/steamvr.vrsettings"
    if ([System.IO.File]::Exists($test_vrsettings_path))
    {
      $vrsettings_path= $test_vrsettings_path
      Write-Host "Found VR Settings path: "$vrsettings_path
      break
    }
    else
    {
      Write-Host "Unable to load path: "$test_vrsettings_path
    }
  }
}
else
{
  Write-Host "Unable to open: "$OpenVRPathsFile
}

if ($vrsettings_path)
{
  $VRSettingsJSON = Get-Content "$vrsettings_path" | Out-String | ConvertFrom-Json
  
  if ($VRSettingsJSON)
  {
    if ($VRSettingsJSON.steamvr)
    {
      if ($VRSettingsJSON.steamvr.activateMultipleDrivers -eq $null)
      {      
        $VRSettingsJSON.steamvr | add-member -Name "activateMultipleDrivers" -value $true -MemberType NoteProperty
        $VRSettingsJSON | ConvertTo-Json | Set-Content "$vrsettings_path"
      }
      elseif ($VRSettingsJSON.steamvr.activateMultipleDrivers -eq $false)
      {      
        $VRSettingsJSON.steamvr.activateMultipleDrivers= $true
        $VRSettingsJSON | ConvertTo-Json | Set-Content "$vrsettings_path"
      }      
      else
      {
        Write-Host "activateMultipleDrivers already set to true."
      }
    }
    else
    {
      $steamvrblock =@"
          {
            "activateMultipleDrivers":true
          }
"@          
      $VRSettingsJSON | add-member -Name "steamvr" -value (Convertfrom-Json $steamvrblock) -MemberType NoteProperty
      $VRSettingsJSON | ConvertTo-Json | Set-Content "$vrsettings_path"
    }
    
    exit 0
  }
  else
  {
    Write-Host "Failed to open: "$vrsettings_path
  }
}
else
{
  Write-Host "Unable to find valid config path in: "$OpenVRPathsFile
}

exit -1