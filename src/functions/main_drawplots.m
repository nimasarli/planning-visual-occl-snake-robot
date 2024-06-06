%% plot Psi
figure(1);

subplot(2,3,1);
h1 = plot(psi_Mat_degree_and_mm(1,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('\theta_{1L}(deg)');

subplot(2,3,2);
h1 = plot(psi_Mat_degree_and_mm(3,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('\theta_{2L}(deg)');

subplot(2,3,3);
h1 = plot(psi_Mat_degree_and_mm(5,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('\theta_{3L}(deg)');

subplot(2,3,4);
h1 = plot(psi_Mat_degree_and_mm(2,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('\delta_{1}(deg)');

subplot(2,3,5);
h1 = plot(psi_Mat_degree_and_mm(4,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('\delta_{2}(deg)');

subplot(2,3,6);
h1 = plot(psi_Mat_degree_and_mm(6,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('\delta_{3}(deg)');

saveas(gcf,'angles','fig');
saveas(gcf,'angles','png');

figure(2);
h2 = plot(psi_Mat_degree_and_mm(7,:));
set(gca,'FontSize',15);
set(h2,'LineWidth',2);
xlabel('Solution index');
ylabel('q_{ins}(mm)');

saveas(gcf,'q_ins','fig');
saveas(gcf,'q_ins','png');

%% plot rate of Psi
figure(3);

subplot(2,3,1);
h1 = plot((180/pi)*psidt_Des_Mat(1,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{\theta}_{1L}(deg/s)$','FontSize',18,'interpreter','latex');

subplot(2,3,2);
h1 = plot((180/pi)*psidt_Des_Mat(3,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{\theta}_{2L}(deg/s)$','FontSize',18,'interpreter','latex');

subplot(2,3,3);
h1 = plot((180/pi)*psidt_Des_Mat(5,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{\theta}_{3L}(deg/s)$','FontSize',18,'interpreter','latex');

subplot(2,3,4);
h1 = plot((180/pi)*psidt_Des_Mat(2,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{\delta}_{1}(deg/s)$','FontSize',18,'interpreter','latex');

subplot(2,3,5);
h1 = plot((180/pi)*psidt_Des_Mat(4,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{\delta}_{2}(deg/s)$','FontSize',18,'interpreter','latex');

subplot(2,3,6);
h1 = plot((180/pi)*psidt_Des_Mat(6,:));
set(gca,'FontSize',15);
set(h1,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{\delta}_{3}(deg/s)$','FontSize',18,'interpreter','latex');

saveas(gcf,'angles_rate','fig');
saveas(gcf,'angles_rate','png');

figure(4);

h2 = plot(1000*psidt_Des_Mat(7,:));
set(gca,'FontSize',15);
set(h2,'LineWidth',2);
xlabel('Solution index');
ylabel('$\dot{q_{ins}}(mm/s)$','FontSize',18,'interpreter','latex');

saveas(gcf,'q_ins_dot','fig');
saveas(gcf,'q_ins_dot','png');

%% plot minimum translational singular values
figure(5);
h4 = plot(sigma_min_Translation_vec);
set(h4,'LineWidth',2);
title('Minimum Translational Singular Value');
xlabel('Solution index');
ylabel('\sigma_{min}');
saveas(gcf,'min_translation_sing_value','fig');
saveas(gcf,'min_translation_sing_value','png');