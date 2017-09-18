clear('conf');

conf = coder.config('exe');
conf.TargetLang = 'c++';
conf.GenerateExampleMain = 'GenerateCodeAndCompile';

codegen -config conf polysync_controller.m
codegen -config conf test_turnsignal.m
codegen -config conf test_shift.m
