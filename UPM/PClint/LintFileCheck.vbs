'------------------------------------------------------------------------------
' This script is responsible of abstract folder and file path for pc lint check
' 
' Parameters
'   1 : path of source folder path
'   2 : path of lint tool folder path
'   3 : search command for check all or search incremental
'
'------------------------------------------------------------------------------
Dim MainPathToSearch
Dim PCLintPath
Dim SearchCommand

If WScript.Arguments.Count = 2 Then
    'Get the arguments
    PCLintPath = WScript.Arguments(0)
    SearchCommand = WScript.Arguments(1)
    'Declare global string to store
    Dim GlobalFilePathString
    Dim GlobalFolderPathString
    Dim LintEnabled
    LintEnabled = 1
    'create file system base
    Set FileSystem = CreateObject("Scripting.FileSystemObject")
    'Get the PClint folder
    Set PCLintFolder = FileSystem.GetFolder(PCLintPath)
    'Get the parent folder
    Set ParentFolder = PCLintFolder.ParentFolder
    GlobalFolderPathString = "-i""" & PCLintPath & """" & vbCrLf
    MainPathToSearch = FileSystem.GetAbsolutePathName(ParentFolder) + "\Source"
    ReleaseFolderPath = FileSystem.GetAbsolutePathName(ParentFolder) + "\Release"
    TiDspFolderPath = FileSystem.GetAbsolutePathName(ParentFolder) + "\TIdsp"
    If FileSystem.FolderExists(MainPathToSearch) = TRUE Then
        GlobalFolderPathString = GlobalFolderPathString & "-i""" & MainPathToSearch & """" & vbCrLf
    else
        MsgBox "Attention: Cannot find Source folder"
        LintEnabled = 0
    end If
    If FileSystem.FolderExists(ReleaseFolderPath) = TRUE Then
        GlobalFolderPathString = GlobalFolderPathString & "-i""" & ReleaseFolderPath & """" & vbCrLf
    else
        MsgBox "Attention: Cannot find Release folder"
        LintEnabled = 0
    end If
    If FileSystem.FolderExists(TiDspFolderPath) = TRUE Then
        GlobalFolderPathString = GlobalFolderPathString & "-i""" & TiDspFolderPath & """" & vbCrLf
    else
        MsgBox "Attention: Cannot find TIdsp folder"
        LintEnabled = 0
    end If
    if LintEnabled = 1 Then
        Set objMainFolder = FileSystem.GetFolder(MainPathToSearch)
        '-----------------------create file------------------------------
        If FileSystem.FileExists(PCLintPath + "\LintSourceFile.txt") = FALSE Then
          FileSystem.CreateTextFile(PCLintPath + "\LintSourceFile.txt")
        End If
        If FileSystem.FileExists(PCLintPath + "\LintHeaderFolder.txt") = FALSE Then
          FileSystem.CreateTextFile(PCLintPath + "\LintHeaderFolder.txt")
        End If
        set PCLintSourceFile = FileSystem.GetFile(PCLintPath + "\LintSourceFile.txt")
        set PCLintHeaderFolder = FileSystem.GetFile(PCLintPath + "\LintHeaderFolder.txt")
        '------------------------search files in folder------------------------------
        'abstract file data to compare
        If FileSystem.FileExists(PCLintPath + "\LintSourceFile.lnt") = FALSE Then
            set PCLintFileDate=FileSystem.GetFile(PCLintPath + "\LintSourceFile.txt")
        else
            set PCLintFileDate=FileSystem.GetFile(PCLintPath + "\LintSourceFile.lnt")
        End If
        PCLintFileLastData = PCLintFileDate.DateLastModified
        PCLintSourceFileDay = year(PCLintFileLastData)*365 + month(PCLintFileLastData)*30 + day(PCLintFileLastData)
        PCLintSourceFileTime = hour(PCLintFileLastData)*3600 + minute(PCLintFileLastData)*60 + second(PCLintFileLastData)
        'search files under main path
        objMainstring = SourceFileSearch(objMainFolder, SearchCommand ,PCLintSourceFileDay, PCLintSourceFileTime)
        GlobalFilePathString = GlobalFilePathString & objMainstring
        'search files under the subfolder of main path
        FolderAndFileSearch MainPathToSearch, SearchCommand, PCLintSourceFileDay, PCLintSourceFileTime
        '------------------------write search result--------------------------------
        Set OutputStream2 = PCLintSourceFile.OpenAsTextStream(2)
        OutputStream2.write GlobalFilePathString
        OutputStream2.Close
        Set OutputStream3 = PCLintHeaderFolder.OpenAsTextStream(2)
        OutputStream3.write GlobalFolderPathString
        OutputStream3.Close
    End if
Else
  MsgBox "Error : Three Arguments is Required!", vbOKOnly + vbCritical , "Lint File Check Tool"
End If
'------------------------------------------------------------------------------
'END OF MAIN ROUTINES
'------------------------------------------------------------------------------
function FolderAndFileSearch(path, searchCommand, fileday, filetime)
    'create file system base
    Set fso=CreateObject("scripting.filesystemobject")
    'Get the folder of the given path
    Set objFolder=fso.GetFolder(path)
    'Get the subfolders under the given folder
    Set objSubFolders=objFolder.Subfolders
    'Search all sub folders
    for each objSubFolder in objSubFolders
        if objSubFolder.name <> ".svn" Then
            'get the subfolder path for regression call
            nowpath=path + "\" + objSubFolder.name
            'Get the files under the folder
            set objFiles=objSubFolder.Files
            'Search specified file type and data
            tempstring = SourceFileSearch(objSubFolder, searchCommand ,fileday, filetime)
            'abstract file path name
            GlobalFilePathString = GlobalFilePathString & tempstring
            'abstract folder path name
            tempstring = "-i""" & nowpath & """" & vbCrLf
            GlobalFolderPathString = GlobalFolderPathString & tempstring
            FolderAndFileSearch nowpath, searchCommand, fileday, filetime
        End if
    next
end function

function SourceFileSearch(folder, searchcommand, fileday, filetime)
    'create file system base
    Set tempFS = CreateObject("scripting.filesystemobject")
    'Get all files under the folder
    set tempobjFiles=folder.Files
    'Search all files
    for each objFile in tempobjFiles
        'Abstract file property
        set tempProp=tempFS.GetFile(objFile)
        'Get the file type
        tempFileType = tempFS.GetExtensionName(objFile)
        'If file type is equal and data is newer than fileData
        if tempFileType = "cpp" or tempFileType = "c" Then
            if searchcommand = "all" Then
                'FileSearch is the return string
                SourceFileSearch = SourceFileSearch & """" & tempFS.GetAbsolutePathName(objFile) & """" & vbCrLf
            elseif searchcommand = "incremental" Then
                'Get the data of the file last modified
                tempData = tempProp.DateLastModified
                tempFileDay = year(tempData)*365 + month(tempData)*30 + day(tempData)
                tempFileTime = hour(tempData)*3600 + minute(tempData)*60 + second(tempData)
                if tempFileDay >= fileday And tempFileTime >= filetime Then
                    'FileSearch is the return string
                    SourceFileSearch = SourceFileSearch & """" & tempFS.GetAbsolutePathName(objFile) & """" & vbCrLf
                    'MsgBox tempFS.GetFileName(objFile)
                end if
            end if
        end if
    next
end function
