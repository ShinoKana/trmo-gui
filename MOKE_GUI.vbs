' Launch the MOKE GUI with NO console window (double-click friendly).
' Uses pythonw.exe so no black cmd box appears. All errors still go to moke_gui.log.
Set sh  = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
here = fso.GetParentFolderName(WScript.ScriptFullName)

' make the bundled science packages importable for any account
sh.Environment("PROCESS")("PYTHONPATH") = here & "\_pypackages"
sh.CurrentDirectory = here

' prefer the system Python 3.12's windowless launcher; fall back to PATH
pyw = "C:\Program Files\Python312\pythonw.exe"
If Not fso.FileExists(pyw) Then pyw = "pythonw"

' 0 = hidden window, False = don't wait
sh.Run """" & pyw & """ """ & here & "\moke_gui.py""", 0, False
