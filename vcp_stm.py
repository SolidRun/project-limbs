#!/usr/bin/python3

#
# Execute STM32 commands VCP_Command_control
#
# Command Option:
# -d  : device (ttyACM2 by default )
# -c  : send command
# -w  : write address (spi flash)
# -r  : read address (spi Flash)
# -D  : data to write (spi Flash)
# example :
# ./vcp_stm_send.py -d ttyACM1 -c voltage
# ./vcp_stm_send.py -d ttyACM1 -c poweron
# ./vcp_stm_send.py -d ttyACM1 -c poweroff
# ./vcp_stm_send.py -d ttyACM1 -c spi_id
# ./vcp_stm_send.py -d ttyACM1 -c spi_wr -w 0x00ff100a -D 0x97
# ./vcp_stm_send.py -d ttyACM1 -c spi_wr -r 0x00ff100a

import os,signal
import subprocess
import time

import argparse
import logging
import sys

# # # # # # # # # # # # #
#   Help functions
# # # # # # # # # # # # #
def str2bytes(value):
    """ Ensures string to bytes conversion """
	# NOTE: eval() is not safe, but the input is provided by a local user, so it could be trusted
    try:
        res = eval(value)
        if type(res) == bytes or type(res) == int:
            return res
    except:
        pass
    raise argparse.ArgumentTypeError('Bytes value expected.')

def send_cmd(command,device):

    resultFile = '/tmp/result'
    resBashCommand = "cat /dev/%s > %s" %(device,resultFile)
    #proc1 = subprocess.Popen(resBashCommand, shell = True)

    execBashCommand = "echo -%s- > /dev/%s" % (command,device)
    # os.popen(execBashCommand).read()
    ## execute with wait the return code
    subprocess.Popen(execBashCommand, shell = True).communicate()

def read_result(FileName):
    # getResultCmd="cat %s" % (FileName)
    # result=os.popen(getResultCmd).read()
    getResultCmd="tail -n 1 %s" % (FileName)
    result=os.popen(getResultCmd).read()
    ## find the last line, change to a file you have
    '''
    with open(FileName) as myfile:
        Lines=list(myfile)
        #print("read_file:",Lines)
        if Lines == []:
            return "Nan"
        result=Lines[-1]
        myfile.close()
    #print("read_file_return:",result)
    '''
    print("read_file_return:",result)
    return result

def poweroff(device,resultFile):
    # check if the machine running
    send_cmd("voltage",device)
    time.sleep(0.5)
    res=read_result(resultFile)
    if not res or res == "0.0V" or res[0] == '0':
        print("The Machine Not Running")
        return 0
    # power off the machine
    print("Power Off after 5 sec.")
    send_cmd("powerbtn_low",device)
    # wait until the Voltage input down to 0V
    count=15 #timeout
    while count > 0:
        res=read_result(resultFile)
        send_cmd("voltage",device)
        # print("-"+res+"-",type(res))
        if res == "0.0V" or res[0] == '0':
            print("Good!!")
            break
        time.sleep(1)
        count-=1
    send_cmd("powerbtn_high",device)
    print("Power Off")

def execute_cmd(cmd,device,cmd_dict):
    '''
    cmd_dict =	{
      "reset":"",
      "poweron": "",
      "poweroff": "",
      "powerbtn_low":"",
      "powerbtn_high":"",
      "vbaton":"",
      "vbatoff":"",
      "current":"str",
      "voltage":"str",
      "spi_sw_stm":"",
      "spi_sw_com":"",
      "spi_test":"bin",
      "spi_id":"bin",
      "spi_uniq_id":"bin",
      "Nan": ""
    }
    '''
    # Validate command format
    if not cmd in cmd_dict:
        print("%s - Command not found "% (cmd))
        print("\nCommands available:")
        for c in cmd_dict.keys():
            print(c)
        return -1

    resultFile = '/tmp/result'
    resBashCommand = "cat /dev/%s > %s" %(device,resultFile)
    proc1 = subprocess.Popen(resBashCommand, shell = True)

    if cmd == "poweroff":
        poweroff(device,resultFile)
        os.killpg(os.getpid(), signal.SIGTERM)
        return 0

    # execute
    #time.sleep(0.2)
    send_cmd(cmd,device)
    # get result
    if cmd_dict[cmd]=="bin":
        time.sleep (1)
        with open(resultFile, mode='rb') as file: # b is important -> binary
            a=file.read()
            hex_str="0x"+a.hex()
            print(hex_str)
            file.close()
    else:
        '''
        getResultCmd="cat %s" % (resultFile)
        res = os.popen(getResultCmd).read()
        '''
        if cmd_dict[cmd]=="str":
            time.sleep(0.8)
            res=read_result(resultFile)
            if res != None :
                print("result: {}".format(res))
            else:
                print("Error result")
        else:
            print("Done")

    # kill the process to get the result (cat $device > $res_f &)
    print("proc1 pid = ",proc1.pid+1)
    # need to fix it because when chose shell=true -> execute_child
    #os.kill(proc1.pid+1, signal.SIGTERM) #or signal.SIGKILL
    # kill a process's child processes in python
    os.killpg(os.getpid(), signal.SIGTERM)
    proc1.kill()

# # # # # # # #
# logger for debug
# # # # # # # #
root_logger = logging.basicConfig(stream=sys.stdout,level=logging.INFO)
local_log=logging.getLogger("STM32-VCP-Command_Control")

# # # # # # # #
#   Main
# # # # # # # #
def main(log_name='stm32_vcp'):
    cmd_dict =	{
      "reset":"",
      "poweron": "",
      "poweroff": "",
      "powerbtn_low":"",
      "powerbtn_high":"",
      "vbaton":"",
      "vbatoff":"",
      "current":"str",
      "voltage":"str",
      "spi_sw_stm":"",
      "spi_sw_com":"",
      "spi_test":"bin",
      "spi_id":"bin",
      "spi_uniq_id":"bin",
      "Nan": ""
    }
    commands_available='| '.join(map(str, cmd_dict.keys()))
    commands_available="Command Operation: "+commands_available

    # Parsing input parameters
    parser = argparse.ArgumentParser(fromfile_prefix_chars='@')
    parser.add_argument('-d',
                        '--usb_device',
                        default='ttyACM2',
                        type=str,
                        help="USB device")

    parser.add_argument('-c',
                        '--cmd',
                        type=str,
                        help=str(commands_available))

    parser.add_argument('-w',
                        '--write_address',
                        type=str2bytes,
                        help="Write Address in w25qxx Spi-Flash")

    parser.add_argument('-D',
                        '--data',
                        type=int,
                        help="Data to Writing")

    parser.add_argument('-r',
                        '--read_address',
                        type=int,
                        help="Read Address, from w25qxx Spi-Flash")

    args = parser.parse_args()

    usb_device = args.usb_device
    cmd = args.cmd
    write_address=args.write_address
    if write_address != None:
        write_address = hex(write_address)
    data = args.data
    if data != None:
        data = hex(data)
    read_address = args.read_address
    if read_address != None:
        read_address = hex(read_address)

    print("usb_device:",usb_device,"cmd:",cmd,"data:",data,"write_address:",write_address,"read_address:",read_address)
    execute_cmd(cmd,usb_device,cmd_dict)

if __name__ == "__main__":
    main()
