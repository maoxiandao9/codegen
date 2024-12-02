function build_computePhaseSlopes_C
    % Reset the target registry to avoid conflicts
    RTW.TargetRegistry.getInstance('reset');

    % Create a coder configuration object for a static library
    cfg = coder.config('lib');
    cfg.GenCodeOnly = true;              % Only generate code
    cfg.TargetLang = 'C';                % Generate C code
    cfg.FilePartitionMethod = 'MapMFileToCFile'; % Generate separate C files per MATLAB function
    cfg.EnableOpenMP = false;            % Disable OpenMP if not needed
    cfg.GenerateReport = true;           % Generate a report for inspection

    % Define input types for code generation
    bufferInputType = coder.typeof(uint16(0), [100, 1]); % Adjust the array size as needed

    % Run code generation
    codegen -config cfg computePhaseSlopes -args {bufferInputType}
end
