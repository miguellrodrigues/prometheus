#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/**
 * @brief Códigos de status retornados pelas funções da biblioteca
 */
typedef enum {
    ENTITYX_OK = 0,            /**< Operação bem sucedida */
    ENTITYX_ERROR = -1,        /**< Erro genérico */
    ENTITYX_MEMORY_ERROR = -2, /**< Erro de alocação de memória */
    ENTITYX_INVALID_PARAM = -3,/**< Parâmetro inválido */
    ENTITYX_NOT_FOUND = -4,    /**< Entidade não encontrada */
    ENTITYX_FILE_ERROR = -5    /**< Erro de operação em arquivo */
} entityx_status_t;

/**
 * @brief Tipos de entidade suportados
 * Adicione novos tipos conforme necessário
 */
typedef enum {
    ENTITY_TYPE_UNKNOWN = 0,  /**< Tipo desconhecido */
    ENTITY_TYPE_USER,         /**< Entidade do tipo usuário */
    ENTITY_TYPE_PRODUCT,      /**< Entidade do tipo produto */
    ENTITY_TYPE_SENSOR,       /**< Entidade do tipo sensor */
    ENTITY_TYPE_MAX          /**< Marcador do número máximo de tipos */
} entityx_type_t;

/**
 * @brief Estrutura que representa uma entrada no gerenciador
 */
typedef struct {
    void* data;                           /**< Ponteiro para os dados da entidade */
    size_t size;                          /**< Tamanho dos dados em bytes */
    entityx_type_t type;                  /**< Tipo da entidade */
    void (*free_fn)(void*);              /**< Função para liberar dados */
    int (*compare_fn)(const void*, const void*); /**< Função de comparação */
} entityx_entry_t;

/**
 * @brief Estrutura principal do gerenciador de entidades
 */
typedef struct {
    entityx_entry_t** entries; /**< Array de ponteiros para entradas */
    size_t count;              /**< Número atual de entidades */
    size_t capacity;           /**< Capacidade total do array */
    const char* file_path;     /**< Caminho do arquivo para persistência */
} entityx_t;

/**
 * @brief Converte tipo de entidade para string
 * @param type Tipo da entidade
 * @return String representando o tipo
 */
const char* entityx_type_to_string(entityx_type_t type);

/**
 * @brief Converte string para tipo de entidade
 * @param type_str String representando o tipo
 * @return Tipo da entidade
 */
entityx_type_t entityx_string_to_type(const char* type_str);

/**
 * @brief Cria um novo gerenciador de entidades
 * @param file_path Caminho do arquivo para persistência
 * @return Ponteiro para o gerenciador ou NULL em caso de erro
 */
entityx_t* entityx_create(const char* file_path);

/**
 * @brief Adiciona uma nova entidade ao gerenciador
 * @param manager Gerenciador de entidades
 * @param data Dados da entidade
 * @param size Tamanho dos dados
 * @param type Tipo da entidade
 * @param free_fn Função para liberar dados (opcional)
 * @param compare_fn Função de comparação (opcional)
 * @return Status da operação
 */
entityx_status_t entityx_add(entityx_t* manager, void* data, size_t size,
                             entityx_type_t type, void (*free_fn)(void*),
                             int (*compare_fn)(const void*, const void*));

/**
 * @brief Remove uma entidade do gerenciador
 * @param manager Gerenciador de entidades
 * @param entry Entrada a ser removida
 * @return Status da operação
 */
entityx_status_t entityx_remove(entityx_t* manager, entityx_entry_t* entry);

/**
 * @brief Busca entidades por um critério específico
 * @param manager Gerenciador de entidades
 * @param key Chave de busca
 * @param compare_fn Função de comparação
 * @param results Array para armazenar resultados
 * @param max_results Tamanho máximo do array de resultados
 * @param found_count Número de resultados encontrados
 * @return Status da operação
 */
entityx_status_t entityx_find(entityx_t* manager, const void* key,
                              int (*compare_fn)(const void*, const void*),
                              entityx_entry_t** results, size_t max_results,
                              size_t* found_count);

/**
 * @brief Busca entidades por tipo
 * @param manager Gerenciador de entidades
 * @param type Tipo da entidade
 * @param results Array para armazenar resultados
 * @param max_results Tamanho máximo do array de resultados
 * @param found_count Número de resultados encontrados
 * @return Status da operação
 */
entityx_status_t entityx_find_by_type(entityx_t* manager, entityx_type_t type,
                                      entityx_entry_t** results, size_t max_results,
                                      size_t* found_count);

/**
 * @brief Salva todas as entidades em arquivo
 * @param manager Gerenciador de entidades
 * @return Status da operação
 */
entityx_status_t entityx_save(entityx_t* manager);

/**
 * @brief Carrega entidades do arquivo
 * @param manager Gerenciador de entidades
 * @param create_fn Função para criar entidade a partir dos dados
 * @param free_fn Função para liberar dados
 * @param compare_fn Função de comparação
 * @return Status da operação
 */
entityx_status_t entityx_load(entityx_t* manager,
                              void* (*create_fn)(entityx_type_t, const void*, size_t),
                              void (*free_fn)(void*),
                              int (*compare_fn)(const void*, const void*));

/**
 * @brief Libera toda a memória do gerenciador
 * @param manager Gerenciador de entidades
 */
void entityx_destroy(entityx_t* manager);

/**
 * @brief Obtém o número de entidades no gerenciador
 * @param manager Gerenciador de entidades
 * @return Número de entidades
 */
size_t entityx_count(entityx_t* manager);

/**
 * @brief Obtém a capacidade atual do gerenciador
 * @param manager Gerenciador de entidades
 * @return Capacidade do gerenciador
 */
size_t entityx_capacity(entityx_t* manager);
