function buildComputeSFO

    RTW.TargetRegistry.getInstance('reset');
    cfg = coder.config('lib');
    cfg.GenCodeOnly = true;
    cfg.EnableMemcpy = true;
    %cfg.TargetLang = 'C++';
    cfg.TargetLang = 'C';
    %cfg.CppInterfaceStyle = "Methods";
    %cfg.CppInterfaceClassName = "computeTOFCalibrated";
    cfg.DataTypeReplacement = "CoderTypedefs";
    cfg.CodeFormattingTool = "Clang-format";
    %cfg.TargetLangStandard = 'C++11 (ISO)';
    cfg.FilePartitionMethod = 'SingleFile';
    cfg.DataTypeReplacement = 'CBuiltIn';
    cfg.EnableAutoParallelizationReporting = false;
    cfg.InstructionSetExtensions = 'None'; 
    cfg.EnableOpenMP = false;

    cfg.LoopUnrollThreshold = 10;
    cfg.SaturateOnIntegerOverflow = false;
    cfg.EnableImplicitExpansion = false;

    cfg.MATLABSourceComments = false;
    cfg.InlineBetweenUserAndMathWorksFunctions = 'Always';
    cfg.InlineBetweenMathWorksFunctions = 'Always';
    cfg.InlineBetweenUserFunctions = 'Always';
    cfg.IndentSize = 4;
    cfg.ColumnLimit = 120;
    cfg.GenerateReport = true;

    tic;
    disp(which(mfilename));
    cd(fileparts(which(mfilename))); % change the MATLAB working directory to the folder of this file.
    %system('rm -rf ./codegen');
    codegen -config cfg computeSFO -args coder.typeof(double(1), [inf 1], [1 0]);
    toc;
end
