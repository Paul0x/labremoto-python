-- phpMyAdmin SQL Dump
-- version 4.6.6deb5
-- https://www.phpmyadmin.net/
--
-- Host: localhost:3306
-- Generation Time: Jan 10, 2021 at 05:45 PM
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
  `label` varchar(100) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `experimento`
--

INSERT INTO `experimento` (`codigo`, `label`) VALUES
(1, 'Apontar Objetivo'),
(2, 'Instruções de Trajetória'),
(3, 'Trajetórias Pré-Definidas');

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
  `cod_sessao` int(11) NOT NULL,
  `cod_experimento` int(11) NOT NULL,
  `parametros` text NOT NULL,
  `dt_inicio` datetime NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=latin1;

--
-- Dumping data for table `sessao_experimento`
--

INSERT INTO `sessao_experimento` (`cod_sessao`, `cod_experimento`, `parametros`, `dt_inicio`) VALUES
(14, 1, 'a', '2021-01-10 16:43:00');

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
-- Indexes for table `sessao`
--
ALTER TABLE `sessao`
  ADD PRIMARY KEY (`codigo`),
  ADD KEY `fk1_sessao` (`matricula`);

--
-- Indexes for table `sessao_experimento`
--
ALTER TABLE `sessao_experimento`
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
-- AUTO_INCREMENT for table `sessao`
--
ALTER TABLE `sessao`
  MODIFY `codigo` int(11) NOT NULL AUTO_INCREMENT, AUTO_INCREMENT=15;
--
-- Constraints for dumped tables
--

--
-- Constraints for table `agenda`
--
ALTER TABLE `agenda`
  ADD CONSTRAINT `fk1_agenda` FOREIGN KEY (`matricula`) REFERENCES `usuario` (`matricula`);

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
