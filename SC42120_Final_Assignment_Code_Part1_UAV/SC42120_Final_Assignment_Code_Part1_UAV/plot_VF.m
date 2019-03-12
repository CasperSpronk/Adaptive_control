close all
if Guid.Strategy==0
    
    figure
    hold on
    plot(SimOut1.xyh.Data(:,2), SimOut1.xyh.Data(:,1),  'LineWidth',1);
    plot(SimOut.PlantData.xyh.Data(:,2), SimOut.PlantData.xyh.Data(:,1),  'LineWidth',1);
    bla = drawCircle(Guid.Loiter.C(2),Guid.Loiter.C(1),Guid.Loiter.R, 'Color', 'k');
    set(bla, 'LineWidth', 2)
    set(bla, 'LineStyle', '--')
    plot(Guid.Loiter.C(2),Guid.Loiter.C(1),'*r', 'LineWidth',1)
    xlabel('[m]'); ylabel('[m]'); grid minor;
    axis square
    title('Bixler Trajectory XY')
    
    figure
    hold on
    plot(SimOut.VF.xtrack_erorr,'LineWidth',1);
    xlabel('[s]'); ylabel('[m]'); grid minor;
    title('e_{py}')
        
    figure
    hold on
    plot(SimOut.VF.Vg_hat,'LineWidth',1);
    hold on
    plot(SimOut.VF.Vg_prime,'LineWidth',1);
    plot(SimOut.VF.V_g,'--','LineWidth',1);
    xlabel('[s]'); ylabel('[m/s]'); grid minor;
    title('Loiter V_g estimation'); legend('Vg_ hat', 'Vg''', 'Vg')
    
    
else
    
    figure
    hold on
    plot(SimOut1.xyh.Data(:,2), SimOut1.xyh.Data(:,1), 'LineWidth',1);
    plot(SimOut.PlantData.xyh.Data(:,2), SimOut.PlantData.xyh.Data(:,1), 'LineWidth',1);
    line = createLine(Guid.SL.Line_Origin(2), Guid.SL.Line_Origin(1),...
        Guid.SL.Line_Slope (2), Guid.SL.Line_Slope (1));
    bla = drawLine(line, 'Color', 'k', 'LineWidth', 1);
    set(bla, 'LineWidth', 2)
    set(bla, 'LineStyle', '--')
    xlabel('[m]'); ylabel('[m]'); grid minor;
    title('Bixler trajectory')
    legend('UAV path', 'Reference line')
    
    figure
    hold on
    plot(SimOut.VF.xtrack_erorr,'LineWidth',1);
    xlabel('[s]'); ylabel('[m]'); grid minor;
    title('e_{py}')
    
    figure
    hold on
    plot(SimOut.VF.Vg_hat,'LineWidth',1);
    hold on
    plot(SimOut.VF.Vg_prime,'LineWidth',1);
    plot(SimOut.VF.V_g, '--', 'LineWidth',1);
    xlabel('[s]'); ylabel('[m/s]'); grid minor;
    title('V_g estimation'); 
    legend('{V}_g'' hat', 'V_g''', 'V_g')
    
    
end

clear bla