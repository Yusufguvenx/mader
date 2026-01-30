% Project MATLAB init script for MADER examples and Gurobi.
% Run from anywhere: init

this_file = mfilename('fullpath');
repo_root = fileparts(this_file);

addpath(fullfile(repo_root, 'matlab_examples'));

license_path = getenv('GRB_LICENSE_FILE');
if isempty(license_path)
    user_profile = getenv('USERPROFILE');
    candidate = fullfile(user_profile, 'gurobi.lic');
    if isfile(candidate)
        setenv('GRB_LICENSE_FILE', candidate);
        license_path = candidate;
    end
end

gurobi_home = getenv('GUROBI_HOME');
if ~isempty(gurobi_home)
    gurobi_matlab = fullfile(gurobi_home, 'matlab');
    if isfolder(gurobi_matlab)
        addpath(gurobi_matlab);
    end
end

if exist('gurobi_setup', 'file') == 2
    gurobi_setup;
end

fprintf('MADER MATLAB init complete.\n');
if ~isempty(license_path)
    fprintf('  GRB_LICENSE_FILE: %s\n', license_path);
else
    fprintf('  GRB_LICENSE_FILE not set (set it if Gurobi fails).\n');
end
if exist('gurobi', 'file') > 0
    fprintf('  Gurobi MATLAB API: available\n');
else
    fprintf('  Gurobi MATLAB API: NOT found on path\n');
end
