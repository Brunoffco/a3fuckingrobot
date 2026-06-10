// Dependência: U8g2 by Oliver
// Repositório oficial: https://github.com/olikraus/u8g2

#include <U8g2lib.h>
#include <Wire.h>
#include <Arduino.h>
#include "Display.h"
#include "Pinagem.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const uint8_t ALTURA_DO_DISPLAY = 64;
const uint8_t LARGURA_DO_DISPLAY = 128;

uint8_t centralizarTxtHorizontal(const char *texto) {
  uint8_t larguraDoTexto = u8g2.getStrWidth(texto);
  return (LARGURA_DO_DISPLAY - larguraDoTexto) / 2;
}

uint8_t centralizarTxtVertical() {
  uint8_t alturaDoTexto = u8g2.getAscent() - u8g2.getDescent();
  return (ALTURA_DO_DISPLAY + alturaDoTexto) / 2;
}

void centralizar(const char *titulo) {
  // Espaçamento a esquerda, espaçamento superior, texto
  u8g2.drawStr(centralizarTxtHorizontal(titulo), centralizarTxtVertical(), titulo);
}

void itemDoMenu(const char *titulo, const char *valor) {
  const uint8_t POSICAO_VERTICAL_TITULO = 20;
  const uint8_t POSICAO_VERTICAL_VALOR = 50;

  // Espaçamento a esquerda, espaçamento superior, texto
  u8g2.drawStr(centralizarTxtHorizontal(titulo), POSICAO_VERTICAL_TITULO, titulo);
  u8g2.drawStr(centralizarTxtHorizontal(valor), POSICAO_VERTICAL_VALOR, valor);
}

void iconesDeNavegacao(bool anterior, bool posterior) {

  u8g2.setFont(u8g2_font_fub14_tr);

  if (posterior) {
    char proximoMenu[] = ">";
    int16_t distanciaEsquerda = LARGURA_DO_DISPLAY - u8g2.getStrWidth(proximoMenu);
    u8g2.drawStr(distanciaEsquerda, centralizarTxtVertical(), proximoMenu);
  }

  if (anterior) {
    char menuAnterior[] = "<";
    u8g2.drawStr(0, centralizarTxtVertical(), menuAnterior);
  }
}

void mensagemDeErro() {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub17_tr);
  char txtErro[] = "Erro!";
  u8g2.drawStr(centralizarTxtHorizontal(txtErro), centralizarTxtVertical(), txtErro);
  u8g2.sendBuffer();
  delay(1500);
}

void display(const char *titulo, const char *valor, bool menuAnterior, bool menuPosterior) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub17_tr);
  itemDoMenu(titulo, valor);
  iconesDeNavegacao(menuAnterior, menuPosterior);
  u8g2.sendBuffer();
}

void display(const char *titulo) {
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_fub17_tr);
  centralizar(titulo);
  u8g2.sendBuffer();
}

void iniciarDisplay() {
  //Wire.begin(SDA_PIN, SCL_PIN);
  u8g2.begin();
}


void limparDisplay() {
  u8g2.clearBuffer();
  u8g2.sendBuffer();
}
