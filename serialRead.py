from __future__ import print_function


def read_sweep_old(file_in):
	sweep = []
	i = 0
	sweep_done = False
	while not sweep_done:
		line = file_in.readline()
		if "sweep_start" in line:
			while not sweep_done:
				line = file_in.readline()
				if "sweep_end" in line:
					sweep_done = True
				else:
					sweep.append(line.split(';')[0]+"\n")
	return sweep


def sweep_to_file(): # old, doesn't work anymore

	file_in = open("/dev/ttyACM0", "r")
	
	sweep = read_sweep_old(file_in)
	file_out = open("sweep_data.txt", "w")
	file_out.writelines(sweep)
	file_out.close()

	file_in.close()


def realtime_sweep():
	file_in=open("/dev/ttyACM0", "r")
	new_sweep=True
	previous_size=0
	direction=1
	prefix_len = 2
	debug = True
	while True:
		reset = False
		sweep_size = 0
		line = file_in.readline()
		if "sweep_start" in line:
			if debug: print("started sweep")
			line = file_in.readline()
			if "sweep_size" in line:
				sweep_size = int(line.split(';')[1])
				if debug: print("sweep_size: "+str(sweep_size))
				line = file_in.readline()
				if "scan_range" in line:
					scan_range = int(line.split(';')[1])
					if debug: print("scan_range: "+str(scan_range))
					if not previous_size == sweep_size:
						truncate_file()
						new_sweep = True
						direction = 1
						append_scan(str(sweep_size))
						append_scan(str(servo_range_to_angle(scan_range)))
					for i in xrange(sweep_size):
						line = file_in.readline()
						if "sample" in line:
							if not new_sweep:
								if direction == 1:
									update_sweep(index=i+prefix_len, sample=line.split(';')[1])
									# if debug: update_sweep(index=i+prefix_len, sample=str(i))
									# if debug: print(i+prefix_len)
								else:
									update_sweep(index=sweep_size+prefix_len-1-i, sample=line.split(';')[1])
									# if debug: update_sweep(index=sweep_size+prefix_len-1-i, sample=str(sweep_size-i-1))
									# if debug: print(sweep_size+prefix_len-1-i)
							else:
								append_scan(line.split(';')[1])
						else:
							if debug: print("resetting.")
							reset = True
							break
					if debug: print("sweep end")
					direction *= -1
					previous_size=sweep_size
					if reset:
						new_sweep = True
					else:
						new_sweep = False


def servo_range_to_angle(servo_range):
	return (9.0*servo_range)/8.0


def truncate_file():
	f = open("sweep_data.txt", "w")
	f.close()


def append_scan(sample):
	f = open("sweep_data.txt", "a")
	f.write(sample+"\n")
	f.close()


def update_sweep(index, sample):
	f = open("sweep_data.txt", "r")
	sweep = f.readlines()
	f.close()
	sweep[index] = sample + "\n"
	f = open("sweep_data.txt", "w")
	f.writelines(sweep)
	f.close()


def printouttty():
	f = open("/dev/ttyACM0", "r")
	while True:
		print(f.readline(), end="")


def menu():
	try:
		while True:
			print("1 - realtime update")
			print("2 - multiview mode")
			print("3 - print raw output")
			print("4 - quit")
			print(">> ", end="")
			usr_input = raw_input() 
			if usr_input == "1":
				realtime_sweep()
				print("running...")
			elif usr_input == "2":
				print("not implemented.")
			elif usr_input == "3":
				printouttty()
			elif usr_input == "4":
				break
			else:
				print("Invalid input. Press any key to continue.")
				raw_input()
	except Exception as e:
		print("Aborting: "+type(e).__name__)
		print(e)
		exit()


menu()
