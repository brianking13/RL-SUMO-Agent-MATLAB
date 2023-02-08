% pyversion('C:/Users/bking/anaconda3/envs/matlab-rl')
% pyenv("Version","3.7")
disp('MATLAB must be restarted to display changes to python scripts')

% Import python file "test.py"
mod = py.importlib.import_module('test');
py.importlib.reload(mod);

% Call function "two" from python file "test.py" (input*2=output)
py.test.two(6)


