print("ESP8266 Server")
wifi.setmode(wifi.STATIONAP);
wifi.ap.config({ssid="DGPS",pwd="12345678"});
wifi.ap.setip({ip="192.168.101.10",netmask="255.255.255.0",gateway="192.168.101.1"})
print("Server IP Address:",wifi.ap.getip())
local roverMAC = "5c:cf:7f:10:d7:4c"
local roverPending = true
roverIP=""
p = ""


tmr.alarm(3, 1000, 1, function() 
           if roverPending then
            uart.write(0,0xB5) 
            uart.write(0,0x62)
            uart.write(0,0xCC) 
            uart.write(0,0xFF)  --Waiting for connection
           end
         end)

sv = net.createServer(net.TCP,600) 
sv:listen(80, function(conn)
    conn:on("receive", function(conn, receivedData) 
    if connected then        
       for i=1,receivedData:len()-1 do
             if receivedData:sub(i,i) == "," then 
                 Str = receivedData:sub(i+1,i+2)
                 Num = tonumber(Str,16) 
                 uart.write(0,Num) -- send received bytes to tha main unit
             end
       end 
      else print("Received Data: " .. receivedData) 
      end
    if receivedData == "Rover_ID001" then 
   for mac,ip in pairs(wifi.ap.getclient()) do
    if mac == roverMAC then roverIP = ip end
end
    tmr.stop(3)
    tmr.start(2)
    roverPending = false
    srv = net.createConnection(net.TCP, 0)
    srv:connect(80,roverIP)
    tmr.alarm(2, 1000, 0, function() 
            srv:send("RS_ID001 accepted")  
            connected = true
            uart.write(0,0xB5) 
            uart.write(0,0x62) 
            uart.write(0,0xCC) 
            uart.write(0,0xFE) 
            uart.write(0,0x01) -- rover connected
            tmr.stop(2)
          end)
       end
    end) 
    conn:on("sent", function(conn) 
    collectgarbage()
   end)
end)

uart.on("data", 0, 
    function(data) 
    if connected then
    if data ~= "*" then p = p .. data end
    if data == "*" then 
    print(p)
    srv:send(p)
    print("data sent")
    p = ""
    end 
    end
end,0)


