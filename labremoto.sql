-- phpMyAdmin SQL Dump
-- version 4.6.6deb5
-- https://www.phpmyadmin.net/
--
-- Host: localhost:3306
-- Generation Time: Jan 11, 2021 at 01:10 PM
-- Server version: 5.7.21-1ubuntu1
-- PHP Version: 7.2.3-1ubuntu1

SET SQL_MODE = "NO_AUTO_VALUE_ON_ZERO";
SET time_zone = "+00:00";


/*!40101 SET @OLD_CHARACTER_SET_CLIENT=@@CHARACTER_SET_CLIENT */;
/*!40101 SET @OLD_CHARACTER_SET_RESULTS=@@CHARACTER_SET_RESULTS */;
/*!40101 SET @OLD_COLLATION_CONNECTION=@@COLLATION_CONNECTION */;
/*!40101 SET NAMES utf8mb4 */;

--
-- Database: `labremoto`
--

-- --------------------------------------------------------

--
-- Table structure for table `agenda`
--

CREATE TABLE `agenda` (
  `codigo` int(11) NOT NULL,
  `matricula` varchar(100) NOT NULL,
  `dt_agendamento` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- --------------------------------------------------------

--
-- Table structure for table `algoritmo_busca`
--

CREATE TABLE `algoritmo_busca` (
  `codigo` int(11) NOT NULL,
  `label` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `algoritmo_busca`
--

INSERT INTO `algoritmo_busca` (`codigo`, `label`) VALUES
(1, 'Astar'),
(2, 'RRT');

-- --------------------------------------------------------

--
-- Table structure for table `configuracoes`
--

CREATE TABLE `configuracoes` (
  `id` int(11) NOT NULL,
  `label` varchar(100) NOT NULL,
  `valor` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `configuracoes`
--

INSERT INTO `configuracoes` (`id`, `label`, `valor`) VALUES
(1, 'duracao_sessao', '25');

-- --------------------------------------------------------

--
-- Table structure for table `experimento`
--

CREATE TABLE `experimento` (
  `codigo` int(11) NOT NULL,
  `label` varchar(100) NOT NULL,
  `descricao` text NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `experimento`
--

INSERT INTO `experimento` (`codigo`, `label`, `descricao`) VALUES
(1, 'Apontar Objetivo', ''),
(2, 'Instruções de Trajetória', ''),
(3, 'Trajetórias Pré-Definidas', '');

-- --------------------------------------------------------

--
-- Table structure for table `experimento_apontar_parametros`
--

CREATE TABLE `experimento_apontar_parametros` (
  `codigo` int(11) NOT NULL,
  `cod_sessao_experimento` int(11) NOT NULL,
  `algoritimo_busca` int(11) NOT NULL,
  `obstaculos` tinyint(1) NOT NULL,
  `pk` float NOT NULL,
  `pd` float NOT NULL,
  `pi` float NOT NULL,
  `tamanho_mapa_busca` int(11) NOT NULL,
  `tamanho_area_seguranca` int(11) NOT NULL,
  `dt_criacacao` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- --------------------------------------------------------

--
-- Table structure for table `experimento_resultados`
--

CREATE TABLE `experimento_resultados` (
  `codigo` int(11) NOT NULL,
  `cod_sessao_experimento` int(11) NOT NULL,
  `pos_x` int(11) NOT NULL,
  `pos_y` int(11) NOT NULL,
  `goal_x` int(11) NOT NULL,
  `goal_y` int(11) NOT NULL,
  `velocidade_linear` float NOT NULL,
  `velocidade_angular` float NOT NULL,
  `experimento_datetime` datetime NOT NULL,
  `dt_criacao` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- --------------------------------------------------------

--
-- Table structure for table `experimento_trajetoria_instrucoes`
--

CREATE TABLE `experimento_trajetoria_instrucoes` (
  `codigo` int(11) NOT NULL,
  `cod_sessao_experimento` int(11) NOT NULL,
  `velocidade_linear` decimal(2,2) NOT NULL,
  `angulo_rotacao` decimal(3,2) NOT NULL,
  `timer` int(11) NOT NULL,
  `dt_criacao` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP,
  `dt_inicializacao` timestamp NULL DEFAULT NULL,
  `dt_finalizacao` timestamp NULL DEFAULT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- --------------------------------------------------------

--
-- Table structure for table `experimento_trajetoria_parametros`
--

CREATE TABLE `experimento_trajetoria_parametros` (
  `codigo` int(11) NOT NULL,
  `cod_sessao_experimento` int(11) NOT NULL,
  `obstaculo` tinyint(1) NOT NULL,
  `kp` float NOT NULL,
  `kd` float NOT NULL,
  `ki` float NOT NULL,
  `dt_criacao` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

-- --------------------------------------------------------

--
-- Table structure for table `sessao`
--

CREATE TABLE `sessao` (
  `codigo` int(11) NOT NULL,
  `matricula` varchar(100) NOT NULL,
  `ativo` tinyint(1) NOT NULL,
  `dt_inicio` datetime NOT NULL,
  `dt_fim` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `sessao`
--

INSERT INTO `sessao` (`codigo`, `matricula`, `ativo`, `dt_inicio`, `dt_fim`) VALUES
(12, '12.2.1165', 0, '2021-01-06 19:31:28', '2021-01-06 19:56:28'),
(13, '12.2.1165', 0, '2021-01-10 16:04:50', '2021-01-10 16:29:50'),
(14, '12.2.1165', 1, '2021-01-10 16:39:47', '2021-01-10 17:04:47');

-- --------------------------------------------------------

--
-- Table structure for table `sessao_experimento`
--

CREATE TABLE `sessao_experimento` (
  `codigo` int(11) NOT NULL,
  `cod_sessao` int(11) NOT NULL,
  `cod_experimento` int(11) NOT NULL,
  `parametros` text NOT NULL,
  `dt_inicio` datetime NOT NULL,
  `ativo` tinyint(1) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `sessao_experimento`
--

INSERT INTO `sessao_experimento` (`codigo`, `cod_sessao`, `cod_experimento`, `parametros`, `dt_inicio`, `ativo`) VALUES
(1, 14, 1, 'a', '2021-01-10 16:43:00', 0);

-- --------------------------------------------------------

--
-- Table structure for table `usuario`
--

CREATE TABLE `usuario` (
  `matricula` varchar(20) NOT NULL,
  `senha` varchar(255) NOT NULL,
  `nome` varchar(255) NOT NULL,
  `email` varchar(255) NOT NULL,
  `tipo_usuario` int(11) NOT NULL DEFAULT '0',
  `dt_criacao` date NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `usuario`
--

INSERT INTO `usuario` (`matricula`, `senha`, `nome`, `email`, `tipo_usuario`, `dt_criacao`) VALUES
('12.2.1165', '460219ddbb37bb46491e96b2e96921d6', 'Paulo Felipe', 'paulof@ufop.edu.br', 1, '2021-01-06');

--
-- Indexes for dumped tables
--

--
-- Indexes for table `agenda`
--
ALTER TABLE `agenda`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `fk1_agenda` (`matricula`);

--
-- Indexes for table `algoritmo_busca`
--
ALTER TABLE `algoritmo_busca`
  ADD PRIMARY KEY (`codigo`);

--
-- Indexes for table `configuracoes`
--
ALTER TABLE `configuracoes`
  ADD PRIMARY KEY (`id`);

--
-- Indexes for table `experimento`
--
ALTER TABLE `experimento`
  ADD PRIMARY KEY (`codigo`);

--
-- Indexes for table `experimento_apontar_parametros`
--
ALTER TABLE `experimento_apontar_parametros`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `pk1_experimento_apontar_parametros` (`cod_sessao_experimento`);

--
-- Indexes for table `experimento_resultados`
--
ALTER TABLE `experimento_resultados`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `pk1_eexperimento_resultados` (`cod_sessao_experimento`);

--
-- Indexes for table `experimento_trajetoria_instrucoes`
--
ALTER TABLE `experimento_trajetoria_instrucoes`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `pk1_experimento_trajetoria_instrucoes` (`cod_sessao_experimento`);

--
-- Indexes for table `experimento_trajetoria_parametros`
--
ALTER TABLE `experimento_trajetoria_parametros`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `pk1_experimento_trajetoria_parametros` (`cod_sessao_experimento`);

--
-- Indexes for table `sessao`
--
ALTER TABLE `sessao`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `fk1_sessao` (`matricula`);

--
-- Indexes for table `sessao_experimento`
--
ALTER TABLE `sessao_experimento`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `fk1_sessao_experimento` (`cod_sessao`),
  ADD KEY `fk2_sessao_experimento` (`cod_experimento`);

--
-- Indexes for table `usuario`
--
ALTER TABLE `usuario`
  ADD PRIMARY KEY (`matricula`);

--
-- AUTO_INCREMENT for dumped tables
--

--
-- AUTO_INCREMENT for table `agenda`
--
ALTER TABLE `agenda`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT;
--
-- AUTO_INCREMENT for table `algoritmo_busca`
--
ALTER TABLE `algoritmo_busca`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=3;
--
-- AUTO_INCREMENT for table `configuracoes`
--
ALTER TABLE `configuracoes`
  MODIFY `id` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=2;
--
-- AUTO_INCREMENT for table `experimento`
--
ALTER TABLE `experimento`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=4;
--
-- AUTO_INCREMENT for table `experimento_apontar_parametros`
--
ALTER TABLE `experimento_apontar_parametros`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT;
--
-- AUTO_INCREMENT for table `experimento_resultados`
--
ALTER TABLE `experimento_resultados`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT;
--
-- AUTO_INCREMENT for table `experimento_trajetoria_instrucoes`
--
ALTER TABLE `experimento_trajetoria_instrucoes`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT;
--
-- AUTO_INCREMENT for table `experimento_trajetoria_parametros`
--
ALTER TABLE `experimento_trajetoria_parametros`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT;
--
-- AUTO_INCREMENT for table `sessao`
--
ALTER TABLE `sessao`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=15;
--
-- AUTO_INCREMENT for table `sessao_experimento`
--
ALTER TABLE `sessao_experimento`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=2;
--
-- Constraints for dumped tables
--

--
-- Constraints for table `agenda`
--
ALTER TABLE `agenda`
  ADD CONSTRAINT `fk1_agenda` FOREIGN KEY (`matricula`) REFERENCES `usuario` (`matricula`);

--
-- Constraints for table `experimento_apontar_parametros`
--
ALTER TABLE `experimento_apontar_parametros`
  ADD CONSTRAINT `pk1_experimento_apontar_parametros` FOREIGN KEY (`cod_sessao_experimento`) REFERENCES `sessao_experimento` (`codigo`);

--
-- Constraints for table `experimento_resultados`
--
ALTER TABLE `experimento_resultados`
  ADD CONSTRAINT `pk1_eexperimento_resultados` FOREIGN KEY (`cod_sessao_experimento`) REFERENCES `sessao_experimento` (`codigo`);

--
-- Constraints for table `experimento_trajetoria_instrucoes`
--
ALTER TABLE `experimento_trajetoria_instrucoes`
  ADD CONSTRAINT `pk1_experimento_trajetoria_instrucoes` FOREIGN KEY (`cod_sessao_experimento`) REFERENCES `sessao_experimento` (`codigo`);

--
-- Constraints for table `experimento_trajetoria_parametros`
--
ALTER TABLE `experimento_trajetoria_parametros`
  ADD CONSTRAINT `pk1_experimento_trajetoria_parametros` FOREIGN KEY (`cod_sessao_experimento`) REFERENCES `sessao_experimento` (`codigo`);

--
-- Constraints for table `sessao`
--
ALTER TABLE `sessao`
  ADD CONSTRAINT `fk1_sessao` FOREIGN KEY (`matricula`) REFERENCES `usuario` (`matricula`);

--
-- Constraints for table `sessao_experimento`
--
ALTER TABLE `sessao_experimento`
  ADD CONSTRAINT `fk1_sessao_experimento` FOREIGN KEY (`cod_sessao`) REFERENCES `sessao` (`codigo`),
  ADD CONSTRAINT `fk2_sessao_experimento` FOREIGN KEY (`cod_experimento`) REFERENCES `experimento` (`codigo`);

/*!40101 SET CHARACTER_SET_CLIENT=@OLD_CHARACTER_SET_CLIENT */;
/*!40101 SET CHARACTER_SET_RESULTS=@OLD_CHARACTER_SET_RESULTS */;
/*!40101 SET COLLATION_CONNECTION=@OLD_COLLATION_CONNECTION */;
