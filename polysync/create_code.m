conf = coder.config('exe');
conf.TargetLang = 'c++';
conf.CustomSource = 'main.cpp';

codegen -config conf polysync_controller.m