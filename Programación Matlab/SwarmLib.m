classdef SwarmLib
    methods(Static)
        
        %Funci�n para obtener de forma autom�tica la IP del robot
        % Pide el nombre del robot del que se desea obtener la IP y el
        % puerto TCP que se utiliza
        % Devuelve la IP en formato de string
        function IP = GetIP(Nombre, Puerto)
            a = 1;
            while a < 256
                IP = ['192.168.1.',num2str(a)];
                try
                    t = tcpclient(IP, Puerto, 'ConnectTimeout', 1);
                    dataOut = [uint8(char('n')),13,10];                  %13 y 10 representan \r y \n
                    write(t, dataOut);
                    pause(0.1);
                    if t.BytesAvailable == 7
                        if char(read(t)) == Nombre
                            a = 256;
                        end
                    end
                    clear t;
                catch
                end
                a = a+1;
            end
        end
        
        %Funci�n para instanciar la conexi�n TCP con el robot.
        % Pide la IP del robot y el puerto TCP.
        % Devuelve el objeto instanciado.
        function TCP = ConexionTCP(IPRobot, Puerto)
            TCP = tcpclient(IPRobot, Puerto, 'Timeout', 1);
        end
        
        %Funci�n para enviar un comando directamente al robot.
        % Pide el comando a enviar (comando que entiende el robot).
        function EnviarComando(Com, ObjTCP)
            dataOut = [uint8(char(Com)),13,10];               %13 y 10 representan \r y \n
            write(ObjTCP, dataOut);
        end
        
        %Funci�n para obtener el nombre de un robot.
        % Pide el objeto TCP de conexi�n con el robot.
        % Devuelve el nombre del robot en string.
        function NombreRobot = GetNombre(ObjTCP)
         
            comando = uint8('n');
            
            write(ObjTCP, [comando,13,10]);
            
            while ObjTCP.BytesAvailable ~= 7
            end
            
            NombreRobot = read(ObjTCP);            
        end
        
        %Funci�n para colocar velocidad a cualquier llanta del robot.
        % Pide la velocidad (entre 0 y 100), la llanta (1 para la derecha y 
        %  2 para la izquierda) y el objeto TCP de conexi�n.
        function SetVelocidad(Vel, Lla, ObjTCP)
            if Vel < 0
                porcentaje = 0;
            elseif Vel > 100
                porcentaje = 100;
            else
                porcentaje = Vel;
            end
            
            numero = porcentaje*4999/100;
            
            mil = floor(numero/1000);
            numero = numero - mil*1000; 
            cen = floor(numero/100);
            numero = numero - cen*100; 
            dec = floor(numero/10);
            
            uni = floor(numero - dec*10);
            
            comando = uint8([mil,cen,dec,uni]+48);
            
            if Lla == 1
                write(ObjTCP, [118,100,comando,13,10]);
            elseif Lla == 2
                write(ObjTCP, [118,105,comando,13,10]);
            end
        end
        
        %Funci�n para colocar el sentido de giro de las llantas.
        % Pide el sentido de las llantas (1 adelante ambas, 2 derecha
        %  adelante e izquierda atr�s, 3 derecha atr�s e izquierda adelante,
        %  4 ambas atr�s.
        function SetSentidos(sentido, ObjTCP)
            if sentido < 1
                sentido = 1;
            elseif sentido > 4
                sentido = 1;
            end
            
            comando = uint8(['s',char(int2str(sentido))]);
            
            write(ObjTCP, [comando,13,10]);
        end
        
        %Funci�n para obtener el sentido de giro de las llantas.
        % Pide el objeto TCP de conexi�n con el robot.
        % Devuelve dos caracteres que representan los sentidos de las
        %  llantas.
        function Sentidos = GetSentidos(ObjTCP)
            comando = uint8('d');
            
            write(ObjTCP, [comando,13,10]);
            
            while ObjTCP.BytesAvailable ~= 2
            end
            
            Sentidos = read(ObjTCP);            
        end
        
        %Funci�n para obtener los pasos o ticks dados por las llantas.
        % Pide el objeto TCP de conexi�n con el robot.
        % Devuelve un vector con 4 n�meros, el primero y segundo son los pasos de la
        %  llanta derecha hacia adelante y hacia atr�s, los otros dos son
        %  los pasos de la llanta izquierda hacia adelante y hacia atr�s
        function Pasos = GetPasos(ObjTCP)
         
            comando = uint8('p');
            
            write(ObjTCP, [comando,13,10]);
            
            while ObjTCP.BytesAvailable ~= 20
            end
            
            dataIn = read(ObjTCP); 
            
            DerechaAdelante = str2double(char(dataIn(1:5)));
            IzquierdaAdelante = str2double(char(dataIn(6:10)));
            DerechaAtras = str2double(char(dataIn(11:15)));
            IzquierdaAtras = str2double(char(dataIn(16:20)));
            
            Pasos = [DerechaAdelante, DerechaAtras, IzquierdaAdelante, IzquierdaAtras];
            
        end
        
        %Funci�n para convertir los pasos o ticks a distancia en metros.
        % Pide la cantidad de pasos a convertir
        % Devuelve la cantidad que equivale en metros
        function Distancia = Pasos2mt(Pasos)
            Distancia = Pasos*42*pi/12/1000;
        end
        
        %Funci�n para convertir los pasos o ticks a distancia en cent�metros.
        % Pide la cantidad de pasos a convertir
        % Devuelve la cantidad que equivale en cent�metros
        function Distancia = Pasos2cm(Pasos)
            Distancia = Pasos*42*pi/12/10;
        end
        
        %Funci�n para obtener los tiempos entre cada tick de las llantas.
        % Pide el objeto TCP de conexi�n con el robot.
        % Devuelve un vector con 2 n�meros, el primero es el tiempo actual
        %  entre pasos de la llanta derecha y el segundo es el tiempo actual
        %  entre pasos de la llanta izquierda. Los tiempos son en segundos.
        function Tiempos = GetTiempos(ObjTCP)
         
            comando = uint8('t');
            
            write(ObjTCP, [comando,13,10]);
            
            while ObjTCP.BytesAvailable ~= 10
            end
            
            dataIn = read(ObjTCP);
            
            Derecha = str2double(char(dataIn(1:5)));
            Izquierda = str2double(char(dataIn(6:10)));
            
            Tiempos = [Derecha, Izquierda]/10000;            
        end
        
        %Funci�n para convertir los tiempos entre ticks a velocidad en cm/s.
        % Pide el tiempo entre ticks.
        % Devuelve la velocidad en cm/s
        function Velocidades = Tiempos2Vel(Tiempo)
            Vel1 = 42*pi/12/10/Tiempo(1);
            Vel2 = 42*pi/12/10/Tiempo(2);
            Velocidades = [Vel1, Vel2];
        end
        
        %Funci�n para obtener las lecturas de los ultras�nicos.
        % Pide el objeto TCP de conexi�n con el robot.
        % Devuelve dos vectores, el primero contiene las distancias en
        %  metros de 292 lecturas, el segundo vector contiene el �ngulo, en
        %  radianes, respectivo a cada lectura.
        function Detecciones = GetDetecciones(ObjTCP)
         
            comando = uint8('r');
            
            write(ObjTCP, [comando,13,10]);
            
            while ObjTCP.BytesAvailable ~= 1168
            end
            
            dataIn = read(ObjTCP);
            
            sensor1 = zeros(1,146);
            sensor2 = zeros(1,146);
            
            for a = 1:8:1168
                n = ((a-1)/8)+1;
                sensor1(n) = str2double(char(dataIn(a:a+3)));
                sensor2(n) = str2double(char(dataIn(a+4:a+7)));
            end
            
            Lecturas = [sensor1,sensor2]*25/147*2.54/100;
            
            Grados = [17:162,197:342]*pi/180;
            
            Detecciones = [Lecturas;Grados];
        end
        
    end
end