function build_InterpolateCSIAndRemoveCSD

    RTW.TargetRegistry.getInstance('reset');
    cfg = coder.config('lib');
    % cfg.CodeReplacementLibrary = 'PermLogic Replacement';
    cfg.GenCodeOnly = true;
    cfg.EnableMemcpy = true;
    cfg.TargetLang = 'C++';
    cfg.CppInterfaceStyle = "Methods";
    cfg.CppInterfaceClassName = "CSIPreprocessor";
    cfg.DataTypeReplacement = "CoderTypedefs";
    cfg.CodeFormattingTool = "Clang-format";
    cfg.TargetLangStandard = 'C++11 (ISO)';
    cfg.FilePartitionMethod = 'SingleFile';
    cfg.DataTypeReplacement = 'CBuiltIn';
    cfg.EnableAutoParallelizationReporting = false;
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
    cd(fileparts(which(mfilename))); % change the MATLAB working directory to the folder of this file.
    system('rm -rf ./codegen');

    inputArray = coder.typeof(uint16(0), [100, 1]); % 根据你的需求设置大小
    output1 = coder.typeof(0.0, [56, 1]); % 确保输出形状和类型一致
    output2 = coder.typeof(0.0, [56, 1]);
    codegen -config cfg computePhaseSlopes -args {inputArray} -o computePhaseSlopes_codegen;

    toc;
end
