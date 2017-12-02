import sys
import numpy as np
import longcontrol as LoC
import adaptivecruise as AC

class Car(object):
  def __init__(self, vLead, dRel, vRel, aLeadK):
    self.vLead = vLead
    self.dRel = dRel
    self.vRel = vRel
    self.aLeadK = aLeadK

def run_controller(enabled, vEgo, vLead1, dRel1, aLeadK1, vLead2, dRel2, aLeadK2):
	# Inputs: 
	# 	vEgo (self velocity in global frame)
	# 	vLead (velocity of lead car in global frame)
	# 	dRel (headway)
	# 	aLeadK (acceleration of lead car in self frame)
	vRel1 = vLead1 - vEgo
	vRel2 = vLead2 - vEgo
	l1 = Car(vLead1, dRel1, vRel1, aLeadK1)
	l2 = Car(vLead2, dRel2, vRel2, aLeadK2)
	acc = AC.AdaptiveCruise()
	acc.update(vEgo, vEgo, l1, l2)
	pid_c = LoC.LongControl()
	# CP = get_params("HONDA CIVIC 2016 TOURING", {})
	# CP.enableGas = True
	final_gas, final_brake = pid_c.update(enabled, vEgo, vEgo, acc.v_target_lead, acc.a_target, acc.jerk_factor, True)
	return final_gas, final_brake

if __name__ == '__main__':

	enabled = bool(sys.argv[1])
	vEgo = float(sys.argv[2])
	vLead1 = float(sys.argv[3])
	dRel1 = float(sys.argv[4])
	aLeadK1 = float(sys.argv[5])
	vLead2 = float(sys.argv[6])
	dRel2 = float(sys.argv[7])
	aLeadK2 = float(sys.argv[8])
	
	final_gas, final_brake = run_controller(enabled, vEgo, vLead1, dRel1, aLeadK1, vLead2, dRel2, aLeadK2)
	sys.stdout.write(str(final_gas))
	sys.stdout.write('\n*\n')
	sys.stdout.write(str(final_brake))
	# a, b, c = squared(x, z)

	# sys.stdout.write(str(a))
	# sys.stdout.write('\n*\n')
	# sys.stdout.write(str(b))
	# sys.stdout.write('\n*\n')
	# sys.stdout.write(str(c))
	# sys.stdout.write(str(squared(x, z)))