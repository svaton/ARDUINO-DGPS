uart.setup(0, 9600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)

print("ESP8266 Client")
gpio.mode(3,gpio.INPUT,gpio.PULLUP)
gpio.read(3)
wifi.sta.disconnect()
wifi.setmode(wifi.STATION) 
wifi.sta.config("DGPS","12345678") -- connecting to server
wifi.sta.connect() 
print("Looking for a connection")
p = ""
r=""
local connected = false
tmr.alarm(0, 1000, 1,function()
    if (wifi.sta.getip() ~= nil) then       
        wifi.sta.setip({ip="192.168.101.11",netmask="255.255.255.0",gateway="192.168.101.1"})
        tmr.stop(0)
        print("CONNECTED -", wifi.sta.getip())
        sv = net.createServer(net.TCP,600) 
        sv:listen(80, function(conn)
            conn:on("receive", function(conn, receivedData) 
                if connected then 
                   for i=1,receivedData:len() do
                        if receivedData:sub(i,i) == "," then 
                            Str = receivedData:sub(i+1,i+2)
                            Num = tonumber(Str,16) 
                            uart.write(0,Num) -- send received bytes to tha main unit
                        end
                   end
                else print("Received Data: " .. receivedData) 
                end
                if receivedData == "RS_ID001 accepted" then  
                print("ready") 
                uart.write(0,0xB5) 
                uart.write(0,0x62) 
                uart.write(0,0xCC) 
                uart.write(0,0xFE) 
                uart.write(0,0x01) -- rover recognized
                connected = true
                
                end       
            end) 
            conn:on("sent", function(conn) 
                collectgarbage()
            end)
        end)
        srv = net.createConnection(net.TCP, 0)
        srv:connect(80,"192.168.101.10")
        tmr.alarm(2, 1000, 1, function() 
            srv:send("Rover_ID001") 
            tmr.stop(2)
          end)
    else
        print("no connection...")
        uart.write(0,0xB5) 
        uart.write(0,0x62)
        uart.write(0,0xCC) 
        uart.write(0,0xFF)  
    end
    
end)

uart.on("data", 0, 
    function(data) 
    if connected then
    p = p .. data
    if p:find("*") ~= nil then 
    srv:send(p)
    p = ""
    end 
    end
end,0)
