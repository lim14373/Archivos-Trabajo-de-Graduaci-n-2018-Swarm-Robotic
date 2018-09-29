import SwarmLib.*

Nombre = 'Robot 1';
Puerto = 5400;

%IPRobot = GetIP(Nombre,Puerto);

IPRobot = '192.168.1.16';

t = ConexionTCP(IPRobot, Puerto);

fprintf('IP del Robot 1: %s \n',IPRobot);
fprintf('1. SetVelocidad\n');
fprintf('2. SetSentidos\n');
fprintf('3. GetVelocidades\n');
fprintf('4. GetSentidos\n');
fprintf('5. GetRecorrido\n');
fprintf('6. Radar\n');
fprintf('7. Nombre Robot\n');
fprintf('8. Salir\n');

p = 1;

while p == 1
    mensaje = input('Opción: ');
    if mensaje ~= 8
        switch mensaje
            case 1
                mensaje2 = input('Llanta derecha o izquierda? (1 o 2): ');
                mensaje = input('Velocidad (0-100): ');
                SetVelocidad(mensaje,mensaje2,t);
            case 2
                mensaje = input('Sentido: ');
                SetSentidos(mensaje,t);
            case 3
                mensaje = Tiempos2Vel(GetTiempos(t));
                fprintf('Derecha: %.2fcm/s\tIzquierda: %.2fcm/s\n',mensaje(1),mensaje(2));
            case 4
                
                mensaje = GetSentidos(t);
                if mensaje(1) == 'f'
                    fprintf('Derecha: Adelante\n');
                else
                    fprintf('Derecha: Atrás\n');
                end
                
                if mensaje(2) == 'f'
                    fprintf('Izquierda: Adelante\n');
                else
                    fprintf('Izquierda: Atrás\n');
                end
            case 5
                mensaje = Pasos2mt(GetPasos(t));
                
                fprintf('\tLlanta Derecha:\n\t\tAdelante: %.2f\tAtrás: %.2f\n',mensaje(1),mensaje(2));
                fprintf('\tLlanta Izquierda:\n\t\tAdelante: %.2f\tAtrás: %.2f\n',mensaje(3),mensaje(4));
            case 6
                mensaje = GetDetecciones(t);
                
                polarplot(mensaje(2,:),mensaje(1,:),'.');
                ax = gca;
                ax.Title.String = 'Escaneo de Objetos';
                ax.RTick = (0:0.05:5);
                rlim([0 .75]);
                d = ax.ThetaDir;
                ax.ThetaDir = 'counterclockwise';
            case 7
                mensaje = GetNombre(t);
                
                fprintf('Nombre: %s\n',mensaje);
            case default
                fprintf('Error');
        end
    else
        p = 0;
        clear t;
    end    
end

